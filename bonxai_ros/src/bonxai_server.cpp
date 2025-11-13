#include "bonxai_server.hpp"

namespace {
template <typename T>
bool update_param(const std::vector<rclcpp::Parameter>& p, const std::string& name, T& value) {
  auto it = std::find_if(p.cbegin(), p.cend(), [&name](const rclcpp::Parameter& parameter) {
    return parameter.get_name() == name;
  });
  if (it != p.cend()) {
    value = it->template get_value<T>();
    return true;
  }
  return false;
}
}  // namespace

namespace bonxai_server {
BonxaiServer::BonxaiServer(const rclcpp::NodeOptions& node_options)
    : Node("bonxai_server_node", node_options) {
  using std::placeholders::_1;
  using std::placeholders::_2;

  {
    world_frame_id_ = declare_parameter("frame_id", "map");
    base_frame_id_ = declare_parameter("base_frame_id", "base_footprint");
  }

  {
    rcl_interfaces::msg::ParameterDescriptor occupancy_min_z_desc;
    occupancy_min_z_desc.description =
        "Minimum height of occupied cells to consider in the final map";
    rcl_interfaces::msg::FloatingPointRange occupancy_min_z_range;
    occupancy_min_z_range.from_value = -100.0;
    occupancy_min_z_range.to_value = 100.0;
    occupancy_min_z_desc.floating_point_range.push_back(occupancy_min_z_range);
    occupancy_min_z_ = declare_parameter("occupancy_min_z", -100.0, occupancy_min_z_desc);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor occupancy_max_z_desc;
    occupancy_max_z_desc.description =
        "Maximum height of occupied cells to consider in the final map";
    rcl_interfaces::msg::FloatingPointRange occupancy_max_z_range;
    occupancy_max_z_range.from_value = -100.0;
    occupancy_max_z_range.to_value = 100.0;
    occupancy_max_z_desc.floating_point_range.push_back(occupancy_max_z_range);
    occupancy_max_z_ = declare_parameter("occupancy_max_z", 100.0, occupancy_max_z_desc);
  }

  {
    rcl_interfaces::msg::ParameterDescriptor max_range_desc;
    max_range_desc.description = "Sensor maximum range";
    rcl_interfaces::msg::FloatingPointRange max_range_range;
    max_range_range.from_value = -1.0;
    max_range_range.to_value = 100.0;
    max_range_desc.floating_point_range.push_back(max_range_range);
    max_range_ = declare_parameter("sensor_model.max_range", -1.0, max_range_desc);
  }

  declare_parameter("resolution", 0.1);
  res_ = get_parameter("resolution").as_double();

  rcl_interfaces::msg::ParameterDescriptor prob_hit_desc;
  prob_hit_desc.description =
      "Probabilities for hits in the sensor model when dynamically building a map";
  rcl_interfaces::msg::FloatingPointRange prob_hit_range;
  prob_hit_range.from_value = 0.5;
  prob_hit_range.to_value = 1.0;
  prob_hit_desc.floating_point_range.push_back(prob_hit_range);
  declare_parameter("sensor_model.hit", 0.7, prob_hit_desc);

  rcl_interfaces::msg::ParameterDescriptor prob_miss_desc;
  prob_miss_desc.description =
      "Probabilities for misses in the sensor model when dynamically building a map";
  rcl_interfaces::msg::FloatingPointRange prob_miss_range;
  prob_miss_range.from_value = 0.0;
  prob_miss_range.to_value = 0.5;
  prob_miss_desc.floating_point_range.push_back(prob_miss_range);
  declare_parameter("sensor_model.miss", 0.4, prob_miss_desc);

  rcl_interfaces::msg::ParameterDescriptor prob_min_desc;
  prob_min_desc.description = "Minimum probability for clamping when dynamically building a map";
  rcl_interfaces::msg::FloatingPointRange prob_min_range;
  prob_min_range.from_value = 0.0;
  prob_min_range.to_value = 1.0;
  prob_min_desc.floating_point_range.push_back(prob_min_range);
  declare_parameter("sensor_model.min", 0.12, prob_min_desc);

  rcl_interfaces::msg::ParameterDescriptor prob_max_desc;
  prob_max_desc.description = "Maximum probability for clamping when dynamically building a map";
  rcl_interfaces::msg::FloatingPointRange prob_max_range;
  prob_max_range.from_value = 0.0;
  prob_max_range.to_value = 1.0;
  prob_max_desc.floating_point_range.push_back(prob_max_range);
  declare_parameter("sensor_model.max", 0.97, prob_max_desc);
  
  // Now get the actual values from parameters (YAML file values if provided)
  const double prob_hit = get_parameter("sensor_model.hit").as_double();
  const double prob_miss = get_parameter("sensor_model.miss").as_double();
  const double thres_min = get_parameter("sensor_model.min").as_double();
  const double thres_max = get_parameter("sensor_model.max").as_double();
  
  RCLCPP_INFO(get_logger(), "Loaded sensor model from parameters: hit=%.3f miss=%.3f min=%.3f max=%.3f", 
              prob_hit, prob_miss, thres_min, thres_max);

  declare_parameter("latch", false);
  latched_topics_ = get_parameter("latch").as_bool();

  // Initialize pause_mapping_ to false (controlled only via service)
  pause_mapping_ = false;

  // Filtering parameters (advanced only; legacy confirm_* removed)
  // Declare parameters first, then get their actual values (which will come from YAML if provided)
  declare_parameter("filter.window_frames", 32);
  declare_parameter("filter.required_observations", 3);  
  declare_parameter("filter.min_neighbor_support", 0);
  declare_parameter("filter.stale_frames", 64);
  declare_parameter("filter.deoccupy_frames", 0);
  declare_parameter("filter.fractional_hits", true);
  
  // Now get the actual values (from YAML file if provided, otherwise defaults)
  auto adv_window_frames = get_parameter("filter.window_frames").as_int();
  auto adv_required_observations = get_parameter("filter.required_observations").as_int();
  auto adv_min_neighbor_support = get_parameter("filter.min_neighbor_support").as_int();
  auto adv_stale_frames = get_parameter("filter.stale_frames").as_int();
  auto adv_deoccupy_frames = get_parameter("filter.deoccupy_frames").as_int();
  auto adv_fractional_hits = get_parameter("filter.fractional_hits").as_bool();
  
  RCLCPP_INFO(get_logger(),
    "Loaded filter config from parameters: window_frames=%ld required=%ld neigh=%ld stale=%ld deocc=%ld fractional=%s",
    static_cast<long>(adv_window_frames), static_cast<long>(adv_required_observations),
    static_cast<long>(adv_min_neighbor_support), static_cast<long>(adv_stale_frames), static_cast<long>(adv_deoccupy_frames), adv_fractional_hits ? "true" : "false");

  // initialize bonxai object & params
  RCLCPP_INFO(get_logger(), "Voxel resolution %f", res_);
  bonxai_ = std::make_unique<BonxaiT>(res_);
  BonxaiT::Options options; // default constructed keeps occupancy_threshold_log
  options.prob_miss_log = bonxai_->logods(prob_miss);
  options.prob_hit_log  = bonxai_->logods(prob_hit);
  options.clamp_min_log = bonxai_->logods(thres_min);
  options.clamp_max_log = bonxai_->logods(thres_max);
  options.window_frames = static_cast<uint8_t>(std::clamp<int>(static_cast<int>(adv_window_frames), 1, 64));
  options.required_observations = static_cast<uint8_t>(std::max<int>(1, static_cast<int>(adv_required_observations)));
  options.min_neighbor_support = static_cast<uint8_t>(adv_min_neighbor_support);
  options.stale_frames = static_cast<uint16_t>(adv_stale_frames);
  options.deoccupy_frames = static_cast<uint16_t>(adv_deoccupy_frames);
  options.fractional_hits = adv_fractional_hits;

  // ROBUSTNESS: Parameter validation and consistency checks
  bool param_warnings = false;

  // Validate probability values
  if (prob_hit <= 0.5 || prob_hit >= 1.0) {
    RCLCPP_WARN(get_logger(), "prob_hit=%.3f outside recommended range (0.5, 1.0)", prob_hit);
    param_warnings = true;
  }
  if (prob_miss <= 0.0 || prob_miss >= 0.5) {
    RCLCPP_WARN(get_logger(), "prob_miss=%.3f outside recommended range (0.0, 0.5)", prob_miss);
    param_warnings = true;
  }
  if (thres_min >= thres_max) {
    RCLCPP_ERROR(get_logger(), "clamp_min (%.3f) >= clamp_max (%.3f) - invalid configuration!", thres_min, thres_max);
    param_warnings = true;
  }

  // Validate filter consistency
  if (options.stale_frames > 0 && options.stale_frames < options.window_frames) {
    RCLCPP_WARN(
      get_logger(),
      "stale_frames (%u) < window_frames (%u) - voxels may be removed before accumulating enough observations",
      options.stale_frames, options.window_frames);
    param_warnings = true;
  }

  if (options.required_observations > options.window_frames) {
    RCLCPP_WARN(
      get_logger(),
      "required_observations (%u) > window_frames (%u) - voxels can never be promoted!",
      options.required_observations, options.window_frames);
    param_warnings = true;
  }

  if (options.min_neighbor_support > 6) {
    RCLCPP_WARN(
      get_logger(),
      "min_neighbor_support (%u) > 6 - extremely strict (max 6 face neighbors available)",
      options.min_neighbor_support);
    param_warnings = true;
  }

  // Add robustness limits if not already set in config
  if (adv_window_frames > 0) { // Check if user provided config
    declare_parameter("filter.max_staging_voxels", 100000);
    declare_parameter("filter.max_tracked_voxels", 200000);
    auto max_staging = get_parameter("filter.max_staging_voxels").as_int();
    auto max_tracked = get_parameter("filter.max_tracked_voxels").as_int();
    options.max_staging_voxels = static_cast<uint32_t>(std::max(0, static_cast<int>(max_staging)));
    options.max_tracked_voxels = static_cast<uint32_t>(std::max(0, static_cast<int>(max_tracked)));
  }

  bonxai_->setOptions(options);

  // Concise startup summary of key parameters
  RCLCPP_INFO(
    get_logger(),
    "Startup params: res=%.3f max_range=%.2f hit=%.2f miss=%.2f clamp=[%.2f..%.2f] "
    "filter{window=%u req=%u neigh=%u stale=%u deocc=%u frac=%s} "
    "limits{staging=%u tracked=%u}%s",
    res_, max_range_, prob_hit, prob_miss, thres_min, thres_max,
    static_cast<unsigned>(options.window_frames), static_cast<unsigned>(options.required_observations),
    static_cast<unsigned>(options.min_neighbor_support),
    static_cast<unsigned>(options.stale_frames), static_cast<unsigned>(options.deoccupy_frames),
    options.fractional_hits ? "on" : "off",
    options.max_staging_voxels, options.max_tracked_voxels,
    param_warnings ? " [WARNINGS PRESENT - check above]" : "");

  if (latched_topics_) {
    RCLCPP_INFO(
        get_logger(),
        "Publishing latched (single publish will take longer, "
        "all topics are prepared)");
  } else {
    RCLCPP_INFO(
        get_logger(),
        "Publishing non-latched (topics are only prepared as needed, "
        "will only be re-published on map change");
  }

  auto qos = latched_topics_ ? rclcpp::QoS{1}.transient_local() : rclcpp::QoS{1};
  point_cloud_pub_ = create_publisher<PointCloud2>("bonxai_point_cloud_centers", qos);

  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  using std::chrono_literals::operator""s;
  point_cloud_sub_.subscribe(this, "cloud_in", rmw_qos_profile_sensor_data);
  tf_point_cloud_sub_ = std::make_shared<tf2_ros::MessageFilter<PointCloud2>>(
      point_cloud_sub_, *tf2_buffer_, world_frame_id_, 5, this->get_node_logging_interface(),
      this->get_node_clock_interface(), 5s);

  tf_point_cloud_sub_->registerCallback(&BonxaiServer::insertCloudCallback, this);

  reset_srv_ =
      create_service<ResetSrv>("~/reset", std::bind(&BonxaiServer::resetSrv, this, _1, _2));

  // service to pause/resume mapping
  pause_srv_ = create_service<PauseSrv>("~/pause_mapping", std::bind(&BonxaiServer::pauseSrv, this,
  _1, _2));

  // set parameter callback
  set_param_res_ =
      this->add_on_set_parameters_callback(std::bind(&BonxaiServer::onParameter, this, _1));
}

void BonxaiServer::insertCloudCallback(const PointCloud2::ConstSharedPtr cloud) {
  if (!cloud) {
    RCLCPP_ERROR(get_logger(), "Received null cloud pointer");
    return;
  }
  if (pause_mapping_) {
    RCLCPP_INFO(get_logger(), "Mapping is paused. Skipping point cloud processing.");
    return;
  }

  const auto start_time = rclcpp::Clock{}.now();

  PCLPointCloud pc;  // input cloud for filtering and ground-detection
  
  try {
    pcl::fromROSMsg(*cloud, pc);
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(get_logger(), "Failed to convert ROS message to PCL: %s", ex.what());
    return;
  }

  if (pc.points.empty()) {
    RCLCPP_WARN(get_logger(), "Received empty point cloud, skipping");
    return;
  }

  // OPTIMIZED: remove NaN and Inf values with early termination
  size_t filtered_index = 0;
  const size_t original_size = pc.points.size();

  for (size_t i = 0; i < original_size; ++i) {
    const auto& point = pc.points[i];
    // OPTIMIZATION: Use likely() hint for branch prediction (most points are valid)
    if (__builtin_expect(std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z), 1)) {
      if (filtered_index != i) {
        pc.points[filtered_index] = point;
      }
      ++filtered_index;
    }
  }

  // Only resize if we actually filtered out points
  if (filtered_index != original_size) {
    pc.resize(filtered_index);
  }

  if (pc.points.empty()) {
    RCLCPP_WARN(get_logger(), "All points filtered out (NaN/Inf), skipping");
    return;
  }

  // Sensor In Global Frames Coordinates
  geometry_msgs::msg::TransformStamped sensor_to_world_transform_stamped;
  try {
    sensor_to_world_transform_stamped = tf2_buffer_->lookupTransform(
        world_frame_id_, cloud->header.frame_id, cloud->header.stamp,
        rclcpp::Duration::from_seconds(1.0));
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    return;
  }

  Eigen::Matrix4f sensor_to_world =
      tf2::transformToEigen(sensor_to_world_transform_stamped.transform).matrix().cast<float>();

  // Transforming Points to Global Reference Frame
  pcl::transformPointCloud(pc, pc, sensor_to_world);

  // Getting the Translation from the sensor to the Global Reference Frame
  const auto& t = sensor_to_world_transform_stamped.transform.translation;

  const pcl::PointXYZ sensor_to_world_vec3(t.x, t.y, t.z);
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!first_cloud_logged_) {
      first_cloud_logged_ = true;
      const auto & opts = bonxai_->options();
      RCLCPP_DEBUG(get_logger(),
        "First cloud arrival: pts=%u window=%u req=%u neigh=%u stale=%u deocc=%u frac=%s",
        cloud->width * cloud->height, opts.window_frames, opts.required_observations,
        opts.min_neighbor_support, opts.stale_frames, opts.deoccupy_frames,
        opts.fractional_hits?"on":"off");
    }
    const auto & opts = bonxai_->options();
    if (opts.window_frames == 0 || opts.window_frames > 64) {
      RCLCPP_ERROR(get_logger(), "Invalid window_frames=%u (corrupted options) — aborting insertion", opts.window_frames);
      return;
    }

    // ROBUSTNESS: Comprehensive validation and error handling
    try {
      // Additional validation before processing
      if (pc.points.size() > 1000000) { // Prevent extremely large point clouds
        RCLCPP_WARN(get_logger(), "Point cloud too large (%zu points), skipping", pc.points.size());
        return;
      }

      RCLCPP_DEBUG(get_logger(), "About to call insertPointCloud with %zu points", pc.points.size());
      
      if (max_range_ >= 0) {
        bonxai_->insertPointCloud(pc.points, sensor_to_world_vec3, max_range_);
      } else {
        bonxai_->insertPointCloud(
            pc.points, sensor_to_world_vec3, std::numeric_limits<double>::infinity());
      }
      
      RCLCPP_DEBUG(get_logger(), "insertPointCloud completed successfully");

      // ROBUSTNESS: Memory pressure monitoring - log warnings when map grows large
      const size_t active_cells = bonxai_->grid().activeCellsCount();
      const size_t mem_usage = bonxai_->grid().memUsage();
      const auto diag = bonxai_->getDiagnostics();

      // Log memory usage every 100 frames or when it exceeds thresholds
      static size_t frame_count = 0;
      static size_t last_logged_frame = 0;
      frame_count++;

      const bool should_log = (frame_count % 100 == 0) ||
        (active_cells > 5000000) ||                        // 5M cells
        (mem_usage > 1000000000) ||                        // 1GB
        (diag.staging_count > 50000) ||
        (diag.tracked_count > 100000);

      if (should_log && (frame_count - last_logged_frame) >= 10) {
        RCLCPP_DEBUG(
          get_logger(),
          "Map stats: cells=%zu mem=%.1fMB staging=%zu tracked=%zu frame=%lu",
          active_cells, mem_usage / 1048576.0,
          diag.staging_count, diag.tracked_count, diag.frame_counter);
        last_logged_frame = frame_count;

        // Warn if approaching concerning levels
        if (active_cells > 8000000) {
          RCLCPP_WARN(
            get_logger(),
            "Map very large (%zu cells) - consider reducing sensor range or increasing resolution",
            active_cells);
        }
        if (mem_usage > 2000000000) { // 2GB
          RCLCPP_WARN(
            get_logger(),
            "High memory usage (%.1f GB) - map may need reset or optimization",
            mem_usage / 1073741824.0);
        }
        if (diag.staging_count > opts.max_staging_voxels* 0.8) {
          RCLCPP_WARN(
            get_logger(),
            "Staging map near limit (%zu / %u) - oldest entries will be purged",
            diag.staging_count, opts.max_staging_voxels);
        }
        if (diag.tracked_count > opts.max_tracked_voxels* 0.8) {
          RCLCPP_WARN(
            get_logger(),
            "Tracked voxels near limit (%zu / %u) - oldest entries will be purged",
            diag.tracked_count, opts.max_tracked_voxels);
        }
      }

    } catch (const std::bad_alloc& ex) {
      RCLCPP_ERROR(get_logger(), "Memory allocation failed in insertPointCloud: %s", ex.what());
      // Try to recover by resetting the map
      bonxai_ = std::make_unique<BonxaiT>(res_);
      auto current_options = bonxai_->options();
      bonxai_->setOptions(current_options);
      return;
    } catch (const std::exception& ex) {
      RCLCPP_ERROR(get_logger(), "Exception in insertPointCloud: %s", ex.what());
      return;
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "Unknown exception in insertPointCloud");
      return;
    }
  }
  double total_elapsed = (rclcpp::Clock{}.now() - start_time).seconds();
  RCLCPP_DEBUG(get_logger(), "Pointcloud insertion in Bonxai done, %f sec)", total_elapsed);

  publishAll(cloud->header.stamp);
}

rcl_interfaces::msg::SetParametersResult BonxaiServer::onParameter(
    const std::vector<rclcpp::Parameter>& parameters) {
  update_param(parameters, "occupancy_min_z", occupancy_min_z_);
  update_param(parameters, "occupancy_max_z", occupancy_max_z_);

  double sensor_model_min{get_parameter("sensor_model.min").as_double()};
  update_param(parameters, "sensor_model.min", sensor_model_min);
  double sensor_model_max{get_parameter("sensor_model.max").as_double()};
  update_param(parameters, "sensor_model.max", sensor_model_max);
  double sensor_model_hit{get_parameter("sensor_model.hit").as_double()};
  update_param(parameters, "sensor_model.hit", sensor_model_hit);
  double sensor_model_miss{get_parameter("sensor_model.miss").as_double()};
  update_param(parameters, "sensor_model.miss", sensor_model_miss);

  // Safely update only sensor model / clamping fields; preserve occupancy threshold and filter settings
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    auto opts = bonxai_->options();
    opts.prob_miss_log = bonxai_->logods(sensor_model_miss);
    opts.prob_hit_log  = bonxai_->logods(sensor_model_hit);
    opts.clamp_min_log = bonxai_->logods(sensor_model_min);
    opts.clamp_max_log = bonxai_->logods(sensor_model_max);
    bonxai_->setOptions(opts);
  }

  bool filter_params_changed = false;
  int tmp_window_frames = get_parameter("filter.window_frames").as_int();
  filter_params_changed |= update_param(parameters, "filter.window_frames", tmp_window_frames);
  int tmp_required_obs = get_parameter("filter.required_observations").as_int();
  filter_params_changed |= update_param(parameters, "filter.required_observations", tmp_required_obs);
  int tmp_min_neigh = get_parameter("filter.min_neighbor_support").as_int();
  filter_params_changed |= update_param(parameters, "filter.min_neighbor_support", tmp_min_neigh);
  int tmp_stale_frames = get_parameter("filter.stale_frames").as_int();
  filter_params_changed |= update_param(parameters, "filter.stale_frames", tmp_stale_frames);
  int tmp_deocc_frames = get_parameter("filter.deoccupy_frames").as_int();
  filter_params_changed |= update_param(parameters, "filter.deoccupy_frames", tmp_deocc_frames);
  bool tmp_fractional = get_parameter("filter.fractional_hits").as_bool();
  filter_params_changed |= update_param(parameters, "filter.fractional_hits", tmp_fractional);

  if (filter_params_changed) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    auto opts = bonxai_->options();
    opts.window_frames = static_cast<uint8_t>(std::clamp(tmp_window_frames, 1, 64));
    opts.required_observations = static_cast<uint8_t>(std::max(1, tmp_required_obs));
    opts.min_neighbor_support = static_cast<uint8_t>(tmp_min_neigh);
    opts.stale_frames = static_cast<uint16_t>(tmp_stale_frames);
    opts.deoccupy_frames = static_cast<uint16_t>(tmp_deocc_frames);
    opts.fractional_hits = tmp_fractional;
    bonxai_->setOptions(opts);
    RCLCPP_INFO(get_logger(),
      "Updated filter config: window_frames=%u required=%u neigh=%u stale=%u deocc=%u fractional=%s",
      opts.window_frames, opts.required_observations,
      opts.min_neighbor_support, opts.stale_frames, opts.deoccupy_frames, opts.fractional_hits ? "true" : "false");
  }

  publishAll(now());

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void BonxaiServer::publishAll(const rclcpp::Time& rostime) {
  const auto start_time = rclcpp::Clock{}.now();
  std::vector<Eigen::Vector3d> bonxai_result;

  // ROBUSTNESS: Keep lock during extraction to prevent race with reset
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!bonxai_) {
      RCLCPP_DEBUG(get_logger(), "publishAll called with null bonxai_");
      return;
    }

    // ROBUSTNESS: Safe extraction of occupied voxels with error handling
    try {
      bonxai_->getOccupiedVoxels(bonxai_result);

      // Validate result size to prevent memory issues
      if (bonxai_result.size() > 10000000) { // 10M voxels max
        RCLCPP_WARN(get_logger(), "Too many occupied voxels (%zu), truncating", bonxai_result.size());
        bonxai_result.resize(10000000);
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Error getting occupied voxels: %s", ex.what());
      return;
    }
  }
  // Lock released here - bonxai_result is now a safe local copy

  if (bonxai_result.size() <= 1) {
    RCLCPP_DEBUG(get_logger(), "Nothing to publish, bonxai is empty");
    return;
  }

  bool publish_point_cloud =
      (latched_topics_ || point_cloud_pub_->get_subscription_count() +
                                  point_cloud_pub_->get_intra_process_subscription_count() >
                              0);

  // init pointcloud for occupied space:
  if (publish_point_cloud) {
  pcl::PointCloud<PCLPoint> pcl_cloud;
  pcl_cloud.reserve(bonxai_result.size());

    for (const auto& voxel : bonxai_result) {  // Directly use occupancy filtered at integration
      if (voxel.z() >= occupancy_min_z_ && voxel.z() <= occupancy_max_z_) {
        pcl_cloud.push_back(PCLPoint(voxel.x(), voxel.y(), voxel.z()));
      }
    }

    // ROBUSTNESS: Validate PCL cloud before publishing
    if (pcl_cloud.empty()) {
      RCLCPP_DEBUG(get_logger(), "All voxels filtered by z-range, nothing to publish");
      return;
    }

    PointCloud2 cloud;
    try {
      pcl::toROSMsg(pcl_cloud, cloud);
      cloud.header.frame_id = world_frame_id_;
      cloud.header.stamp = rostime;
      point_cloud_pub_->publish(cloud);
      RCLCPP_DEBUG(get_logger(), "Published occupancy grid with %ld voxels", pcl_cloud.points.size());
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Error publishing point cloud: %s", ex.what());
    }
  }
}

bool BonxaiServer::resetSrv(
    const std::shared_ptr<ResetSrv::Request>, const std::shared_ptr<ResetSrv::Response>) {
  const auto rostime = now();
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    // Preserve current options before recreating the map
    auto current_options = bonxai_->options();
    bonxai_ = std::make_unique<BonxaiT>(res_);
    bonxai_->setOptions(current_options);
    // No need to reset accessor - new map instance creates fresh internal state
  }

  RCLCPP_INFO(get_logger(), "Cleared Bonxai map and reset all internal structures");
  publishAll(rostime);

  return true;
}

// Service callback to control mapping pause/resume
bool BonxaiServer::pauseSrv(
    const std::shared_ptr<PauseSrv::Request> request, const std::shared_ptr<PauseSrv::Response> response) {
  // Set the pause_mapping_ flag based on service request
  pause_mapping_ = request->data;

  // Log the state and respond
  if (pause_mapping_) {
    RCLCPP_INFO(get_logger(), "Mapping paused.");
    response->message = "Mapping is now paused.";
  } else {
    RCLCPP_INFO(get_logger(), "Mapping resumed.");
    response->message = "Mapping has resumed.";
  }

  response->success = true;
  return true;
}


}  // namespace bonxai_server

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(bonxai_server::BonxaiServer)
