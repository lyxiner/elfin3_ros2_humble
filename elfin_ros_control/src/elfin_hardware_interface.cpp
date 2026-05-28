#include <elfin_ros_control/elfin_hw_interface.h>
#include <elfin_ros_control/visibility_control.h>
#include <boost/shared_ptr.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <cmath>
#include <exception>
#include <hardware_interface/handle.hpp>
 
 
namespace elfin_hardware_interface
{
 
  using CallbackReturn = hardware_interface::CallbackReturn;
 
  // ---------------- helpers (unchanged logic) ----------------
  bool ElfinHWInterface::isModuleMoving(int module_num)
  {
    std::vector<double> previous_pos;
    std::vector<double> last_pos;
 
    previous_pos.resize(2);
    last_pos.resize(2);
 
    int32_t count1, count2;
 
    module_infos_[module_num].client_ptr->getActPosCounts(count1, count2);
    previous_pos[0] = count1 / module_infos_[module_num].axis1.count_rad_factor;
    previous_pos[1] = count2 / module_infos_[module_num].axis2.count_rad_factor;
 
    usleep(10000);
 
    module_infos_[module_num].client_ptr->getActPosCounts(count1, count2);
    last_pos[0] = count1 / module_infos_[module_num].axis1.count_rad_factor;
    last_pos[1] = count2 / module_infos_[module_num].axis2.count_rad_factor;
 
    for (unsigned int i = 0; i < previous_pos.size(); i++)
    {
      if (fabs(last_pos[i] - previous_pos[i]) > motion_threshold_)
      {
        return true;
      }
    }
    return false;
  }
 
  bool ElfinHWInterface::setGroupPosMode(const std::vector<int> &module_no)
  {
    for (unsigned int j = 0; j < module_no.size(); j++)
    {
      boost::mutex::scoped_lock pre_switch_flags_lock(*pre_switch_mutex_ptrs_[module_no[j]]);
      pre_switch_flags_[module_no[j]] = true;
      pre_switch_flags_lock.unlock();
    }
 
    usleep(10000);
 
    for (unsigned int j = 0; j < module_no.size(); j++)
    {
      module_infos_[module_no[j]].client_ptr->setPosMode();
    }
 
    usleep(10000);
 
    for (unsigned int j = 0; j < module_no.size(); j++)
    {
      if (!module_infos_[module_no[j]].client_ptr->inPosMode())
      {
        RCLCPP_ERROR(n_->get_logger(), "module[%i]: set position mode failed", module_no[j]);
        for (unsigned int k = 0; k < module_no.size(); k++)
        {
          boost::mutex::scoped_lock pre_switch_flags_lock(*pre_switch_mutex_ptrs_[module_no[k]]);
          pre_switch_flags_[module_no[k]] = false;
          pre_switch_flags_lock.unlock();
        }
        return false;
      }
    }
    return true;   // <-- 修复原代码漏写的返回
  }
 
  void ElfinHWInterface::release_resources()
  {
    // 只释放 EtherCAT 资源, 不动 module_infos_ 骨架。
    // 那是 on_init() 一次性建好的, 整个 plugin 生命周期内只有 size/name 是稳定的, 用于
    // 让 export_*_interfaces() 看见的接口表始终一致。
    for (auto * drv : ethercat_drivers_)
    {
      if (drv != nullptr) { delete drv; }
    }
    ethercat_drivers_.clear();
 
    if (em != nullptr)
    {
      delete em;
      em = nullptr;
    }
    for (auto & mi : module_infos_) { mi.client_ptr = nullptr; }
    pre_switch_flags_.clear();
    pre_switch_mutex_ptrs_.clear();
    initialized_ = false;
    first_pass_ = true;
  }
 
  // ---------------- Lifecycle: on_init ----------------
  // Humble 下 export_state_interfaces() 会在 on_configure() 之前被 ResourceManager 调用,
  // 所以 module_infos_ 的“骨架”——大小、关节名、状态/命令存储——必须在 on_init() 里就建好。
  // EtherCAT 相关字段(client_ptr/reduction_ratio/count_zero 等)留到 on_configure() 再填。
  CallbackReturn ElfinHWInterface::on_init(const hardware_interface::HardwareInfo & info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }
 
    n_ = rclcpp::Node::make_shared("elfin_hw");
 
    if (!n_->has_parameter("elfin_ethercat_drivers"))
    {
      n_->declare_parameter<std::vector<std::string>>(
        "elfin_ethercat_drivers", std::vector<std::string>({"elfin"}));
    }
    if (!n_->has_parameter("elfin_ethernet_name"))
    {
      n_->declare_parameter<std::string>("elfin_ethernet_name", "eth0");
    }
 
    // ===== 预分配 module_infos_, 让 export_*_interfaces() 看到完整骨架 =====
    if (info_.joints.size() % 2 != 0)
    {
      RCLCPP_FATAL(n_->get_logger(),
        "ElfinHWInterface expects an even number of joints (2 axes per EtherCAT module), got %zu.",
        info_.joints.size());
      return CallbackReturn::ERROR;
    }
    const size_t num_modules = info_.joints.size() / 2;
    module_infos_.assign(num_modules, ModuleInfo{});
 
    auto init_axis = [](AxisInfo & ax, const std::string & jname) {
      ax.name                   = jname;
      ax.ec_axis_idx            = 0;     // 0 = 未绑定; on_configure 会改成 1 或 2
      ax.reduction_ratio        = 1.0;
      ax.count_rad_factor       = 1.0;
      ax.count_rad_per_s_factor = 1.0;
      ax.count_Nm_factor        = 1.0;
      ax.count_zero             = 0;
      ax.axis_position_factor   = 1.0;
      ax.axis_torque_factor     = 1.0;
      ax.position               = 0.0;
      ax.velocity               = 0.0;
      ax.effort                 = 0.0;
      ax.position_cmd           = 0.0;
      ax.velocity_cmd           = 0.0;
      ax.vel_ff_cmd             = 0.0;
      ax.effort_cmd             = 0.0;
    };
 
    for (size_t i = 0; i < num_modules; ++i)
    {
      module_infos_[i].client_ptr = nullptr;
      init_axis(module_infos_[i].axis1, info_.joints[2 * i].name);
      init_axis(module_infos_[i].axis2, info_.joints[2 * i + 1].name);
    }
 
    RCLCPP_INFO(n_->get_logger(),
      "ElfinHWInterface on_init() done. URDF joints=%zu, allocated modules=%zu",
      info_.joints.size(), num_modules);
    return CallbackReturn::SUCCESS;
  }
 
  // ---------------- Lifecycle: on_configure ----------------
  // 连 EtherCAT, 按 joint name 把 EtherCAT 数据填进 on_init 预分配好的 module_infos_ 槽位。
  // 不再重建 module_infos_, 因为 export_*_interfaces() 已经在 on_init 之后被调用过、
  // 注册的接口名就是 URDF joint name; 在这里必须保持那套命名不变。
  CallbackReturn ElfinHWInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    std::vector<std::string> elfin_driver_names_default = {"elfin"};
    std::string ethercat_name_default = "eth0";
 
    n_->get_parameter_or("elfin_ethercat_drivers", elfin_driver_names_, elfin_driver_names_default);
    std::string ethernet_name;
    n_->get_parameter_or("elfin_ethernet_name", ethernet_name, ethercat_name_default);
 
    // 只清 EtherCAT 资源, 不动 module_infos_ 骨架(那是 on_init 建的)
    for (auto * drv : ethercat_drivers_) { if (drv) { delete drv; } }
    ethercat_drivers_.clear();
    if (em) { delete em; em = nullptr; }
    for (auto & mi : module_infos_) { mi.client_ptr = nullptr; }
 
    ethercat_drivers_.resize(elfin_driver_names_.size());
    try
    {
      em = new elfin_ethercat_driver::EtherCatManager(ethernet_name);
    }
    catch (const std::exception & e)
    {
      RCLCPP_FATAL(n_->get_logger(), "EtherCatManager init failed: %s", e.what());
      return CallbackReturn::ERROR;
    }
    for (size_t i = 0; i < ethercat_drivers_.size(); ++i)
    {
      ethercat_drivers_[i] =
        new elfin_ethercat_driver::ElfinEtherCATDriver(em, elfin_driver_names_[i], n_);
    }
 
    // 工具 lambda: 按 joint name 在 module_infos_ 里找到对应的 AxisInfo
    auto find_axis = [&](const std::string & jname) -> AxisInfo * {
      for (auto & mi : module_infos_)
      {
        if (mi.axis1.name == jname) { return &mi.axis1; }
        if (mi.axis2.name == jname) { return &mi.axis2; }
      }
      return nullptr;
    };
    auto find_module = [&](const std::string & jname) -> ModuleInfo * {
      for (auto & mi : module_infos_)
      {
        if (mi.axis1.name == jname || mi.axis2.name == jname) { return &mi; }
      }
      return nullptr;
    };
 
    // 按 EtherCAT yaml 顺序遍历每根轴, 按 joint name 填到正确槽位。
    for (size_t di = 0; di < ethercat_drivers_.size(); ++di)
    {
      auto * drv = ethercat_drivers_[di];
      for (size_t ci = 0; ci < drv->getEtherCATClientNumber(); ++ci)
      {
        auto * client = drv->getEtherCATClientPtr(ci);
        const std::string name_a1 = drv->getJointName(2 * ci);
        const std::string name_a2 = drv->getJointName(2 * ci + 1);
 
        ModuleInfo * mod_a1 = find_module(name_a1);
        ModuleInfo * mod_a2 = find_module(name_a2);
        AxisInfo * ax_a1 = find_axis(name_a1);
        AxisInfo * ax_a2 = find_axis(name_a2);
 
        if (!mod_a1 || !mod_a2 || !ax_a1 || !ax_a2)
        {
          RCLCPP_FATAL(n_->get_logger(),
            "EtherCAT joint name(s) not found in URDF: '%s', '%s'."
            " Check that URDF and elfin_drivers.yaml use the same joint names.",
            name_a1.c_str(), name_a2.c_str());
          return CallbackReturn::ERROR;
        }
        if (mod_a1 != mod_a2)
        {
          RCLCPP_FATAL(n_->get_logger(),
            "EtherCAT slave's two axes ('%s' and '%s') must belong to the same URDF module pair."
            " Check URDF joint ordering vs elfin_drivers.yaml slave grouping.",
            name_a1.c_str(), name_a2.c_str());
                    return CallbackReturn::ERROR;
        }

        mod_a1->client_ptr = client;

        // ec_axis_idx 1 表示这个 AxisInfo 槽位对应物理 EC slave 上的 1 号轴;
        // 2 表示对应 2 号轴。read/write 按此 dispatch 到 getAxis1*/getAxis2*。
        ax_a1->ec_axis_idx            = 1;
        ax_a1->reduction_ratio        = drv->getReductionRatio(2 * ci);
        ax_a1->axis_position_factor   = drv->getAxisPositionFactor(2 * ci);
        ax_a1->count_zero             = drv->getCountZero(2 * ci);
        ax_a1->axis_torque_factor     = drv->getAxisTorqueFactor(2 * ci);

        ax_a2->ec_axis_idx            = 2;
        ax_a2->reduction_ratio        = drv->getReductionRatio(2 * ci + 1);
        ax_a2->axis_position_factor   = drv->getAxisPositionFactor(2 * ci + 1);
        ax_a2->count_zero             = drv->getCountZero(2 * ci + 1);
        ax_a2->axis_torque_factor     = drv->getAxisTorqueFactor(2 * ci + 1);
      }
    }

    // 计算 count_*_factor (与原 Foxy 实现一致)
    for (auto & mi : module_infos_)
    {
      mi.axis1.count_rad_factor =
        mi.axis1.reduction_ratio * mi.axis1.axis_position_factor / (2 * M_PI);
      mi.axis1.count_rad_per_s_factor = mi.axis1.count_rad_factor / 750.3;
      mi.axis1.count_Nm_factor =
        mi.axis1.axis_torque_factor / mi.axis1.reduction_ratio;

      mi.axis2.count_rad_factor =
        mi.axis2.reduction_ratio * mi.axis2.axis_position_factor / (2 * M_PI);
      mi.axis2.count_rad_per_s_factor = mi.axis2.count_rad_factor / 750.3;
      mi.axis2.count_Nm_factor =
        mi.axis2.axis_torque_factor / mi.axis2.reduction_ratio;
    }

    // 检查所有 slot 都拿到了 client_ptr (没拿到说明 URDF 里有 yaml 里没有的 joint name)
    for (size_t i = 0; i < module_infos_.size(); ++i)
    {
      if (module_infos_[i].client_ptr == nullptr)
      {
        RCLCPP_FATAL(n_->get_logger(),
          "URDF module %zu (joints '%s' / '%s') has no matching EtherCAT slave.",
          i, module_infos_[i].axis1.name.c_str(), module_infos_[i].axis2.name.c_str());
        return CallbackReturn::ERROR;
      }
    }

    // 读一次初始位置, 处理 ±π 折回。注意按 ec_axis_idx 取数据,不能假设 axis1/axis2 ↔ EC1/EC2。
    auto read_raw_pos = [](AxisInfo & ax, elfin_ethercat_driver::ElfinEtherCATClient * c) -> int32_t {
      return (ax.ec_axis_idx == 1) ? c->getAxis1PosCnt() : c->getAxis2PosCnt();
    };

    for (auto & mi : module_infos_)
    {
      int32_t pc1 = read_raw_pos(mi.axis1, mi.client_ptr);
      double t1 = (pc1 - mi.axis1.count_zero) / mi.axis1.count_rad_factor;
      if (t1 >= M_PI)       { mi.axis1.count_zero += mi.axis1.count_rad_factor * 2 * M_PI; }
      else if (t1 < -M_PI)  { mi.axis1.count_zero -= mi.axis1.count_rad_factor * 2 * M_PI; }
      mi.axis1.position = -1.0 * (pc1 - mi.axis1.count_zero) / mi.axis1.count_rad_factor;

      int32_t pc2 = read_raw_pos(mi.axis2, mi.client_ptr);
      double t2 = (pc2 - mi.axis2.count_zero) / mi.axis2.count_rad_factor;
      if (t2 >= M_PI)       { mi.axis2.count_zero += mi.axis2.count_rad_factor * 2 * M_PI; }
      else if (t2 < -M_PI)  { mi.axis2.count_zero -= mi.axis2.count_rad_factor * 2 * M_PI; }
      mi.axis2.position = -1.0 * (pc2 - mi.axis2.count_zero) / mi.axis2.count_rad_factor;
    }

    pre_switch_flags_.assign(module_infos_.size(), false);
    pre_switch_mutex_ptrs_.resize(module_infos_.size());
    for (unsigned int i = 0; i < pre_switch_mutex_ptrs_.size(); ++i)
    {
      pre_switch_mutex_ptrs_[i] = boost::shared_ptr<boost::mutex>(new boost::mutex);
    }

    RCLCPP_INFO(n_->get_logger(),
                "ElfinHWInterface on_configure() done. modules=%zu (EtherCAT connected)",
                module_infos_.size());
    return CallbackReturn::SUCCESS;
  }

  // ---------------- export interfaces (signature unchanged) ----------------
  std::vector<hardware_interface::StateInterface> ElfinHWInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < module_infos_.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        module_infos_[i].axis1.name, hardware_interface::HW_IF_POSITION, &module_infos_[i].axis1.position));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        module_infos_[i].axis2.name, hardware_interface::HW_IF_POSITION, &module_infos_[i].axis2.position));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
        module_infos_[i].axis1.name, hardware_interface::HW_IF_VELOCITY, &module_infos_[i].axis1.velocity));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        module_infos_[i].axis2.name, hardware_interface::HW_IF_VELOCITY, &module_infos_[i].axis2.velocity));
    }
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> ElfinHWInterface::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.reserve(module_infos_.size());
    for (unsigned int i = 0; i < module_infos_.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        module_infos_[i].axis1.name, hardware_interface::HW_IF_POSITION, &module_infos_[i].axis1.position_cmd));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        module_infos_[i].axis2.name, hardware_interface::HW_IF_POSITION, &module_infos_[i].axis2.position_cmd));

      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        module_infos_[i].axis1.name, hardware_interface::HW_IF_VELOCITY, &module_infos_[i].axis1.velocity_cmd));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        module_infos_[i].axis2.name, hardware_interface::HW_IF_VELOCITY, &module_infos_[i].axis2.velocity_cmd));
    }
    return command_interfaces;
  }

  // ---------------- mode switch ----------------
  return_type ElfinHWInterface::prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces)
  {
    auto cur_interface = [](const std::string & interface) {
      return interface.find(hardware_interface::HW_IF_POSITION) != std::string::npos;
    };
    int64_t num_stop_cur_interfaces =
      std::count_if(stop_interfaces.begin(), stop_interfaces.end(), cur_interface);
    if (num_stop_cur_interfaces == 6)
    {
      pos_interface_claimed = false;
    }
    else if (num_stop_cur_interfaces != 0)
    {
      RCLCPP_FATAL(n_->get_logger(),
                   "Expected %d pos interfaces to stop, but got %ld instead.",
                   6, num_stop_cur_interfaces);
      std::string error_string = "Invalid number of pos interfaces to stop, Expected ";
      error_string += std::to_string(6);
      throw std::invalid_argument(error_string);
    }

    int64_t num_start_cur_interfaces =
      std::count_if(start_interfaces.begin(), start_interfaces.end(), cur_interface);
    if (num_start_cur_interfaces == 6)
    {
      pos_interface_claimed = true;
    }
    else if (num_start_cur_interfaces != 0)
    {
      RCLCPP_FATAL(n_->get_logger(),
                   "Expected %d pos interfaces to start, but got %ld instead.",
                   6, num_start_cur_interfaces);
      std::string error_string = "Invalid number of pos interfaces to start, Expected ";
      error_string += std::to_string(6);
      throw std::invalid_argument(error_string);
    }
    return return_type::OK;
  }

  return_type ElfinHWInterface::perform_command_mode_switch(
    const std::vector<std::string> &, const std::vector<std::string> &)
  {
    if (pos_interface_claimed && !pos_interface_running)
    {
      pos_interface_running = true;
    }
    else if (pos_interface_running && !pos_interface_claimed)
    {
      pos_interface_running = false;
    }
    return return_type::OK;
  }

  // ---------------- Lifecycle: activate / deactivate / cleanup / shutdown ----------------
  CallbackReturn ElfinHWInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // 进入 active 之前先 read 一次，并把 position_cmd 同步成当前 position，
    // 防止激活瞬间往零位冲过去。
    read(n_->now(), rclcpp::Duration::from_seconds(0.0));
    for (size_t i = 0; i < module_infos_.size(); i++)
    {
      module_infos_[i].axis1.position_cmd = module_infos_[i].axis1.position;
      module_infos_[i].axis2.position_cmd = module_infos_[i].axis2.position;
    }
    initialized_ = true;
    first_pass_  = false;
    RCLCPP_INFO(n_->get_logger(), "ElfinHWInterface activated.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn ElfinHWInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(n_->get_logger(), "ElfinHWInterface deactivated.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn ElfinHWInterface::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(n_->get_logger(), "ElfinHWInterface cleaning up...");
    release_resources();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn ElfinHWInterface::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(n_->get_logger(), "ElfinHWInterface shutting down.");
    release_resources();
    return CallbackReturn::SUCCESS;
  }

  // ---------------- read / write (new signature: time + period) ----------------
  return_type ElfinHWInterface::read(
    const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
  {
    read_update_time_ = time;
    rclcpp::spin_some(n_);

    // 用 lambda 按 ec_axis_idx 把"从哪个物理 EC 轴拿原始 count"封装起来
    auto read_axis = [](AxisInfo & ax, elfin_ethercat_driver::ElfinEtherCATClient * c) {
      int32_t pos_count;
      int16_t vel_count, trq_count;
      if (ax.ec_axis_idx == 1) {
        pos_count = c->getAxis1PosCnt();
        vel_count = c->getAxis1VelCnt();
        trq_count = c->getAxis1TrqCnt();
      } else {
        pos_count = c->getAxis2PosCnt();
        vel_count = c->getAxis2VelCnt();
        trq_count = c->getAxis2TrqCnt();
      }
      int32_t pos_diff = pos_count - ax.count_zero;
      ax.position = -1.0 * pos_diff   / ax.count_rad_factor;
      ax.velocity = -1.0 * vel_count  / ax.count_rad_per_s_factor;
      ax.effort   = -1.0 * trq_count  / ax.count_Nm_factor;
    };

    for (auto & mi : module_infos_)
    {
      read_axis(mi.axis1, mi.client_ptr);
      read_axis(mi.axis2, mi.client_ptr);
    }

    if (first_pass_ && !initialized_)
    {
      for (auto & mi : module_infos_)
      {
        mi.axis1.position_cmd = mi.axis1.position;
        mi.axis2.position_cmd = mi.axis2.position;
      }
      initialized_ = true;
    }

    return return_type::OK;
  }

  return_type ElfinHWInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    auto write_pos = [](AxisInfo & ax, elfin_ethercat_driver::ElfinEtherCATClient * c) {
      double cnt = -1.0 * ax.position_cmd * ax.count_rad_factor + ax.count_zero;
      if (ax.ec_axis_idx == 1) {
        c->setAxis1PosCnt(static_cast<int32_t>(cnt));
      } else {
        c->setAxis2PosCnt(static_cast<int32_t>(cnt));
      }
    };

    for (size_t i = 0; i < module_infos_.size(); i++)
    {
      auto & mi = module_infos_[i];

      if (!mi.client_ptr->inPosBasedMode())
      {
        mi.axis1.position_cmd = mi.axis1.position;
        mi.axis2.position_cmd = mi.axis2.position;
      }

      write_pos(mi.axis1, mi.client_ptr);
      write_pos(mi.axis2, mi.client_ptr);

      bool is_preparing_switch;
      {
        boost::mutex::scoped_lock pre_switch_flags_lock(*pre_switch_mutex_ptrs_[i]);
        is_preparing_switch = pre_switch_flags_[i];
      }
      if (!is_preparing_switch)
      {
        // 速度前馈、力矩通道目前没启用; 保持注释跟原代码一致, 后续要用时按 ec_axis_idx dispatch
      }
    }
    return return_type::OK;
  }

}  // namespace elfin_hardware_interface

PLUGINLIB_EXPORT_CLASS(elfin_hardware_interface::ElfinHWInterface,
                       hardware_interface::SystemInterface)
