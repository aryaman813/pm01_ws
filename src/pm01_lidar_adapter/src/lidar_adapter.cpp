
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

// Shorter aliases for ROS 2 message types.
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

// ============================================================
// SECTION 1: Internal point representation
// ============================================================
//
// Each original PM01 PointCloud2 message contains approximately
// 96 points.
//
// We read those points into this structure before combining
// multiple messages into a 100 ms scan.
//
// timestamp_ns is the original absolute acquisition timestamp,
// expressed in nanoseconds.
//
// We keep it as a 64-bit integer to avoid losing precision
// during the scan aggregation process.
//
struct TimedPoint
{
    float x;
    float y;
    float z;

    float intensity;

    uint8_t tag;
    uint8_t line;

    uint64_t timestamp_ns;
};

// ============================================================
// SECTION 2: ROS 2 adapter node
// ============================================================

class LidarAdapter : public rclcpp::Node
{
public:
    LidarAdapter()
        : Node("pm01_lidar_adapter")
    {
        // 100 ms per scan = approximately 10 Hz.
        scan_duration_ns_ = 100000000ULL;

        // Allow a small amount of packet reordering.
        // A scan is published only after timestamps advance
        // at least 5 ms beyond its ending boundary.
        //
        // This introduces approximately 5 ms of additional
        // acquisition-time latency.
        reorder_delay_ns_ = 5000000ULL;

        // Reset the scan buffer if there is an unusually
        // large discontinuity in the LiDAR timestamps.
        timestamp_jump_ns_ = 10000000000ULL;

        // The PM01 publisher previously reported RELIABLE QoS
        // with a history depth of 256.
        //
        // Match that reliability setting for the input.
        auto input_qos =
            rclcpp::QoS(rclcpp::KeepLast(256)).reliable();

        subscription_ = create_subscription<PointCloud2>(
            "/livox/lidar",
            input_qos,
            [this](PointCloud2::ConstSharedPtr msg)
            {
                lidar_callback(msg);
            });

        // Publish a standard ROS 2 PointCloud2 message.
        //
        // SensorDataQoS uses BEST_EFFORT reliability,
        // which is common for point-cloud consumers.
        publisher_ = create_publisher<PointCloud2>(
            "/livox/lidar_modified",
            rclcpp::SensorDataQoS());

        RCLCPP_INFO(
            get_logger(),
            "PM01 LiDAR adapter started");

        RCLCPP_INFO(
            get_logger(),
            "Input: /livox/lidar");

        RCLCPP_INFO(
            get_logger(),
            "Output: /livox/lidar_modified (10 Hz)");
    }

private:
    // ========================================================
    // SECTION 3: ROS publishers, subscribers and scan state
    // ========================================================

    rclcpp::Subscription<PointCloud2>::SharedPtr subscription_;

    rclcpp::Publisher<PointCloud2>::SharedPtr publisher_;

    uint64_t scan_duration_ns_;
    uint64_t reorder_delay_ns_;
    uint64_t timestamp_jump_ns_;

    // Points waiting to be grouped into completed scans.
    std::vector<TimedPoint> pending_points_;

    // Latest point acquisition timestamp observed.
    uint64_t latest_timestamp_ns_ = 0;

    // Beginning of the current 100 ms scan window.
    uint64_t scan_start_ns_ = 0;

    // Whether a scan window has been initialized.
    bool initialized_ = false;

    // Keep the original frame ID instead of assuming
    // every source message uses "livox_frame".
    std::string frame_id_;

    // ========================================================
    // SECTION 4: PointCloud2 field lookup
    // ========================================================
    //
    // PointCloud2 stores point data as raw bytes.
    //
    // The fields array describes where x, y, z, intensity,
    // tag, line and timestamp are stored in each point.
    //
    // We locate fields by name instead of assuming offsets.
    //

    const PointField* find_field(
        const PointCloud2& msg,
        const std::string& name) const
    {
        for (const auto& field : msg.fields)
        {
            if (field.name == name)
            {
                return &field;
            }
        }

        return nullptr;
    }

    // Read a value from the raw point bytes.
    //
    // memcpy avoids potential alignment problems that can
    // occur when casting raw byte pointers directly.
    template<typename T>
    T read_field(
        const uint8_t* point,
        const PointField* field) const
    {
        T value;

        std::memcpy(
            &value,
            point + field->offset,
            sizeof(T));

        return value;
    }

    // ========================================================
    // SECTION 5: Validate the incoming PointCloud2 format
    // ========================================================
    //
    // Expected PM01 field types:
    //
    // x, y, z    -> FLOAT32
    // intensity  -> FLOAT32
    // tag        -> UINT8
    // line       -> UINT8
    // timestamp  -> FLOAT64
    //
    // The source point_step was previously measured as 26.
    // However, we use the actual offsets from the message.
    //

    bool validate_fields(
        const PointCloud2& msg,
        const PointField*& fx,
        const PointField*& fy,
        const PointField*& fz,
        const PointField*& fi,
        const PointField*& ftag,
        const PointField*& fline,
        const PointField*& ft)
    {
        fx = find_field(msg, "x");
        fy = find_field(msg, "y");
        fz = find_field(msg, "z");

        fi = find_field(msg, "intensity");

        ftag = find_field(msg, "tag");
        fline = find_field(msg, "line");

        ft = find_field(msg, "timestamp");

        if (!fx || !fy || !fz || !fi ||
            !ftag || !fline || !ft)
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Missing required PM01 PointCloud2 fields");

            return false;
        }

        if (msg.is_bigendian)
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Big-endian PointCloud2 is not supported");

            return false;
        }

        const bool types_valid =
            fx->datatype == PointField::FLOAT32 &&
            fy->datatype == PointField::FLOAT32 &&
            fz->datatype == PointField::FLOAT32 &&
            fi->datatype == PointField::FLOAT32 &&
            ftag->datatype == PointField::UINT8 &&
            fline->datatype == PointField::UINT8 &&
            ft->datatype == PointField::FLOAT64;

        if (!types_valid)
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Unexpected PointCloud2 field datatypes");

            return false;
        }

        // Verify that each field fits inside point_step.
        const bool offsets_valid =
            fx->offset + 4 <= msg.point_step &&
            fy->offset + 4 <= msg.point_step &&
            fz->offset + 4 <= msg.point_step &&
            fi->offset + 4 <= msg.point_step &&
            ftag->offset + 1 <= msg.point_step &&
            fline->offset + 1 <= msg.point_step &&
            ft->offset + 8 <= msg.point_step;

        if (!offsets_valid)
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "PointCloud2 field offsets are invalid");

            return false;
        }

        return true;
    }

    // ========================================================
    // SECTION 6: LiDAR subscription callback
    // ========================================================
    //
    // This function runs whenever a new PointCloud2 message
    // arrives on /livox/lidar.
    //
    // It:
    // 1. Validates the incoming message.
    // 2. Reads all points.
    // 3. Extracts the original absolute timestamps.
    // 4. Adds the points to the aggregation buffer.
    //

    void lidar_callback(PointCloud2::ConstSharedPtr msg)
    {
        const PointField* fx = nullptr;
        const PointField* fy = nullptr;
        const PointField* fz = nullptr;
        const PointField* fi = nullptr;
        const PointField* ftag = nullptr;
        const PointField* fline = nullptr;
        const PointField* ft = nullptr;

        if (!validate_fields(
                *msg,
                fx, fy, fz, fi, ftag, fline, ft))
        {
            return;
        }

        // The PM01 source cloud is expected to be unorganized:
        // height = 1, width = number of points.
        if (msg->height != 1 || msg->width == 0)
        {
            return;
        }

        // Validate the raw data buffer before reading.
        if (msg->point_step == 0 ||
            msg->row_step <
                static_cast<uint64_t>(msg->width) * msg->point_step ||
            msg->data.size() < msg->row_step)
        {
            return;
        }

        if (frame_id_.empty())
        {
            frame_id_ = msg->header.frame_id;
        }
        else if (frame_id_ != msg->header.frame_id)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "LiDAR frame_id changed; resetting adapter buffer");

            pending_points_.clear();
            initialized_ = false;
            latest_timestamp_ns_ = 0;
            frame_id_ = msg->header.frame_id;
        }

        std::vector<TimedPoint> incoming;

        incoming.reserve(msg->width);

        for (uint32_t i = 0; i < msg->width; ++i)
        {
            const uint8_t* raw_point =
                msg->data.data() +
                static_cast<size_t>(i) * msg->point_step;

            TimedPoint point;

            point.x = read_field<float>(raw_point, fx);
            point.y = read_field<float>(raw_point, fy);
            point.z = read_field<float>(raw_point, fz);

            point.intensity =
                read_field<float>(raw_point, fi);

            point.tag =
                read_field<uint8_t>(raw_point, ftag);

            point.line =
                read_field<uint8_t>(raw_point, fline);

            const double timestamp =
                read_field<double>(raw_point, ft);

            // Ignore points containing invalid coordinates
            // or invalid timestamps.
            if (!std::isfinite(point.x) ||
                !std::isfinite(point.y) ||
                !std::isfinite(point.z) ||
                !std::isfinite(point.intensity) ||
                !std::isfinite(timestamp) ||
                timestamp <= 0.0 ||
                timestamp >=
                    static_cast<double>(
                        std::numeric_limits<int64_t>::max()))
            {
                continue;
            }

            // Convert the absolute nanosecond timestamp
            // from FLOAT64 to UINT64.
            //
            // The original FLOAT64 representation already
            // has limited nanosecond precision at Unix-epoch
            // magnitudes. Conversion cannot restore that
            // precision, but relative millisecond offsets
            // remain useful for FAST-LIO.
            point.timestamp_ns =
                static_cast<uint64_t>(timestamp);

            incoming.push_back(point);
        }

        if (incoming.empty())
        {
            return;
        }

        // Sort the points from this incoming message
        // by their acquisition timestamps.
        std::sort(
            incoming.begin(),
            incoming.end(),
            [](const TimedPoint& a, const TimedPoint& b)
            {
                return a.timestamp_ns < b.timestamp_ns;
            });

        // Add this message's points to the pending buffer.
        pending_points_.insert(
            pending_points_.end(),
            incoming.begin(),
            incoming.end());

        // Record the newest timestamp observed.
        latest_timestamp_ns_ = std::max(
            latest_timestamp_ns_,
            incoming.back().timestamp_ns);

        if (!initialized_)
        {
            // Align scan boundaries to 100 ms intervals
            // in the LiDAR acquisition-time domain.
            scan_start_ns_ =
                (incoming.front().timestamp_ns /
                 scan_duration_ns_) * scan_duration_ns_;

            initialized_ = true;
        }

        // Protect against a large timestamp discontinuity.
        //
        // For example, if a sensor restarts or its clock jumps,
        // the old buffer must not be combined with new data.
        if (latest_timestamp_ns_ > scan_start_ns_ &&
            latest_timestamp_ns_ - scan_start_ns_ >
                timestamp_jump_ns_)
        {
            RCLCPP_WARN(
                get_logger(),
                "Large LiDAR timestamp jump detected; resetting buffer");

            pending_points_.clear();
            initialized_ = false;
            latest_timestamp_ns_ = 0;

            return;
        }

        publish_completed_scans();
    }

    // ========================================================
    // SECTION 7: Identify completed 100 ms scans
    // ========================================================
    //
    // The latest observed timestamp is used as a watermark.
    //
    // We wait for an additional 5 ms of acquisition time
    // before publishing a completed scan. This allows some
    // late packets to arrive before the scan is finalized.
    //
    // The buffer is sorted by timestamp before publishing.
    //

    void publish_completed_scans()
    {
        if (!initialized_ || pending_points_.empty())
        {
            return;
        }

        // Wait until the current scan has ended and the
        // reordering allowance has elapsed.
        while (latest_timestamp_ns_ >= scan_start_ns_ &&
               latest_timestamp_ns_ - scan_start_ns_ >=
                   scan_duration_ns_ + reorder_delay_ns_)
        {
            const uint64_t scan_end_ns =
                scan_start_ns_ + scan_duration_ns_;

            // Sort the entire pending buffer to handle
            // packets that arrive slightly out of order.
            std::sort(
                pending_points_.begin(),
                pending_points_.end(),
                [](const TimedPoint& a, const TimedPoint& b)
                {
                    return a.timestamp_ns < b.timestamp_ns;
                });

            // Identify points belonging to the current scan.
            auto boundary = std::lower_bound(
                pending_points_.begin(),
                pending_points_.end(),
                scan_end_ns,
                [](const TimedPoint& point, uint64_t timestamp)
                {
                    return point.timestamp_ns < timestamp;
                });

            // Drop any points older than the current window.
            auto first = std::lower_bound(
                pending_points_.begin(),
                boundary,
                scan_start_ns_,
                [](const TimedPoint& point, uint64_t timestamp)
                {
                    return point.timestamp_ns < timestamp;
                });

            if (first != boundary)
            {
                publish_scan(first, boundary);
            }

            // Remove the completed scan and stale points.
            pending_points_.erase(
                pending_points_.begin(),
                boundary);

            // Advance to the next acquisition-time window.
            scan_start_ns_ = scan_end_ns;
        }
    }

    // ========================================================
    // SECTION 8: Construct and publish PointCloud2
    // ========================================================
    //
    // Output fields:
    //
    // x            FLOAT32
    // y            FLOAT32
    // z            FLOAT32
    // intensity    FLOAT32
    // reflectivity FLOAT32
    // time         FLOAT32
    // tag          UINT8
    // line         UINT8
    //
    // "time" stores milliseconds relative to scan_start_ns_.
    //
    // FAST-LIO's modified MID360 handler will copy this
    // value into PointType.curvature for motion compensation.
    //

    void publish_scan(
        std::vector<TimedPoint>::const_iterator begin,
        std::vector<TimedPoint>::const_iterator end)
    {
        const size_t point_count =
            static_cast<size_t>(std::distance(begin, end));

        if (point_count == 0)
        {
            return;
        }

        PointCloud2 output;

        // Use the LiDAR acquisition time for the scan header,
        // not the current computer clock.
        output.header.stamp.sec =
            static_cast<int32_t>(
                scan_start_ns_ / 1000000000ULL);

        output.header.stamp.nanosec =
            static_cast<uint32_t>(
                scan_start_ns_ % 1000000000ULL);

        output.header.frame_id = frame_id_;

        output.height = 1;
        output.width = static_cast<uint32_t>(point_count);

        output.is_bigendian = false;
        output.is_dense = false;

        auto add_field = [&](const std::string& name,
                             uint32_t offset,
                             uint8_t datatype)
        {
            PointField field;

            field.name = name;
            field.offset = offset;
            field.datatype = datatype;
            field.count = 1;

            output.fields.push_back(field);
        };

        add_field("x", 0, PointField::FLOAT32);
        add_field("y", 4, PointField::FLOAT32);
        add_field("z", 8, PointField::FLOAT32);

        add_field("intensity", 12, PointField::FLOAT32);
        add_field("reflectivity", 16, PointField::FLOAT32);

        add_field("time", 20, PointField::FLOAT32);

        add_field("tag", 24, PointField::UINT8);
        add_field("line", 25, PointField::UINT8);

        // 26 bytes of actual fields, padded to 28 bytes
        // for convenient memory alignment.
        output.point_step = 28;

        output.row_step =
            output.width * output.point_step;

        output.data.resize(output.row_step);

        size_t index = 0;

        for (auto it = begin; it != end; ++it, ++index)
        {
            const TimedPoint& point = *it;

            uint8_t* dst =
                output.data.data() +
                index * output.point_step;

            // Convert nanoseconds to milliseconds relative
            // to the beginning of this 100 ms scan.
            const float relative_time_ms =
                static_cast<float>(
                    static_cast<double>(
                        point.timestamp_ns - scan_start_ns_) /
                    1000000.0);

            std::memcpy(dst + 0, &point.x, sizeof(float));
            std::memcpy(dst + 4, &point.y, sizeof(float));
            std::memcpy(dst + 8, &point.z, sizeof(float));

            std::memcpy(
                dst + 12,
                &point.intensity,
                sizeof(float));

            // Keep the same measurement under reflectivity
            // for compatibility with FAST-LIO's MID360 path.
            std::memcpy(
                dst + 16,
                &point.intensity,
                sizeof(float));

            std::memcpy(
                dst + 20,
                &relative_time_ms,
                sizeof(float));

            std::memcpy(
                dst + 24,
                &point.tag,
                sizeof(uint8_t));

            std::memcpy(
                dst + 25,
                &point.line,
                sizeof(uint8_t));
        }

        // Publish the completed 100 ms scan.
        publisher_->publish(output);
    }
};

// ============================================================
// SECTION 9: Main function
// ============================================================
//
// Initialize ROS 2, create the adapter node, and keep it
// running until the user stops the process with Ctrl+C.
//

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LidarAdapter>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
