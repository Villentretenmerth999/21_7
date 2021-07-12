#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
// #include <franka_example_controllers/iden_dynamics.h>
#include <franka_example_controllers/CTC_controller.h>
//#include <franka_example_controllers/pseudo_inversion.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
//#include <rclcpp/rclcpp.hpp> // ros2

#include <stdint.h> // uint64_t;
#include <chrono> // time related
#include <ctime> // get_date_time

#include <iostream>
#include <fstream>
#include <string>

#include <cmath>
#include <memory>
using namespace std;
using namespace Eigen;
using namespace std::chrono; // time related

namespace franka_example_controllers {
/* code */
bool CTC_controller::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) {
  string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CTC_cotroller: Could not read parameter arm_id");
    return false;
  }

  vector<string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "CTC_controller: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("CTC_controller: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ =
        make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CTC_controller: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("CTC_controller: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ =
        make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CTC_controller: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("CTC_controller: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("CTC_controller: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

 // CTC_out_publisher_.init(node_handle, "CTC_out_data", 4);
//error:request for member ‘init’ in     which is of non-class type ‘int’


  q_desired.setZero();
  dq_desired.setZero();
  ddq_desired.setZero();
  k_gain.setZero();
  d_gain.setZero();

  pos_0.setZero();
  ori_0.coeffs() << 0.0, 0.0, 0.0, 1.0;
  jacobian_0.setZero(); 

  return true;
}

void CTC_controller::starting(const ros::Time& /*time*/) {
  // set the initial state
  franka::RobotState initial_state = state_handle_->getRobotState();
  Map<Matrix<double, 7, 1>> q_initial(initial_state.q.data());            //map的用法

  q_initial_ = q_initial;

  Affine3d initial_transform(Matrix4d::Map(initial_state.O_T_EE.data())); //map的另一种用法
  pos_0 = initial_transform.translation();
  ori_0 = Quaterniond(initial_transform.linear());
  array<double, 42> initial_jacobian = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  Map<Matrix<double, 6, 7>> jacobian0(initial_jacobian.data());
  jacobian_0 = jacobian0; 

  // Date and Time
    date_time_ = get_date_time();

    // Directory
   // !node_handle->setParam<std::string>("directory", "~/log/panda");
    //directory_ = this->getParam("directory").as_string();

    directory_ ="../log/panda";

    if (directory_.front() == '~') {
        directory_.erase(directory_.begin());
        directory_ = std::string(getenv("HOME")) + directory_;
    }
    
    if (directory_.back() == '/') {
        directory_ = directory_ + date_time_ + std::string("/");
    } else {
        directory_ = directory_ + std::string("/") + date_time_ + std::string("/");
    }

    std::string cmd = std::string("mkdir -p ") + directory_;
   // if (!system(cmd.data())) {
   //    RCLCPP_INFO(this->get_logger(),std::string("Folder created: ") + directory_);
   //  ROS_INFO_STREAM("Folder created: "<< directory_);
    //}
    // Files
    pos_.open(directory_ + std::string("pos_.csv"));
    vel_.open(directory_ + std::string("vel_.csv"));
    tau_.open(directory_ + std::string("tau_.csv"));
    time_.open(directory_ + std::string("time_.csv"));


    pos_ << "timestamp" << "," << "q1" << "," << "q2" << "," << "q3" << "," << "q4" << "," << "q5" << "," << "q6"<<"," <<  "q7"<< std::endl;
    vel_ << "timestamp" << ","  << "qd1" << "," << "qd2" << "," << "qd3" << "," << "qd4" << "," << "qd5" << "," << "qd6"<<"," <<  "qd7" << std::endl;
    tau_ << "timestamp" << "," <<"tau1" << "," << "tau2" << "," << "tau3" << "," << "tau4" << "," << "tau5" << "," << "tau6"<<"," <<  "tau7"<< std::endl;
    time_ << "time" << std::endl;

  elapsed_time_ = ros::Duration(0.0);
}

void CTC_controller::update(const ros::Time& /*time*/, const ros::Duration& period) {
  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  array<double, 7> coriolis_array = model_handle_->getCoriolis();
  array<double, 7> gravity_array = model_handle_->getGravity();
  array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  array<double, 49> mass_array = model_handle_->getMass();

  // convert to Eigen
  Map<Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Map<Matrix<double, 7, 1>> gravity(gravity_array.data());
  Map<Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Map<Matrix<double, 7, 7>> mass(mass_array.data());
  Map<Matrix<double, 7, 1>> q(robot_state.q.data());
  Map<Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Map<Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());  // desired linkside torque without gravity,NOLINT(readability-identifier-naming)
  Affine3d transform(Matrix4d::Map(robot_state.O_T_EE.data()));
  Vector3d position(transform.translation());
  Quaterniond orientation(transform.linear());

  elapsed_time_ += period;

  //give the trajectory in cartesian
  Vector3d pos_d;
  Quaterniond ori_d;
  pos_d = pos_0;
  ori_d = ori_0;
  Matrix<double, 6, 1> x_d_dot;
  Matrix<double, 6, 1> x_d_ddot;

  // if (elapsed_time_.toSec() >= 10 && elapsed_time_.toSec() <=30)
  //   {
      // double delta_z = 0.1 * (1 - cos(M_PI / 5 * elapsed_time_.toSec()));
      // pos_d[2] -= delta_z;

      // x_d_dot.head(3) << 0.0, 0.0, -0.1 * M_PI / 5 * sin(M_PI / 5 * elapsed_time_.toSec());
      // x_d_ddot.head(3) << 0.0, 0.0, -0.1 * M_PI / 5 * M_PI / 5 * cos(M_PI / 5 * elapsed_time_.toSec());
      // x_d_dot.tail(3) << 0.0, 0.0, 0.0;
      // x_d_ddot.tail(3) << 0.0, 0.0, 0.0;




      

      //calculate jacobian_diff inverse
      // Matrix<double, 6, 7> jacobian_diff = Jacobian_diff(jacobian_0, jacobian, period.toSec());
      // jacobian_0 = jacobian;
      // MatrixXd jacobian_pinv;
      // pseudoInverse(jacobian, jacobian_pinv);  //damped Singular value decomposition

      
      // dq_desired = jacobian_pinv * x_d_dot;  //just a inverse kinematic test
      // ddq_desired = jacobian_pinv * (x_d_ddot - jacobian_diff * dq_desired);
      // q_desired = q_initial_ + dq_desired * period.toSec();

    // }da
    // else
    // {
    //   // rd_pos[2] -= 0.1 * (1 - cos(M_PI / 5 * 30));
    //   rd_dot.head(3) << 0.0, 0.0, 0;
    //   rd_ddot.head(3) << 0.0, 0.0, 0;
    //   rd_dot.tail(3) << 0.0, 0.0, 0.0;
    //   rd_ddot.tail(3) << 0.0, 0.0, 0.0;
    // }


 Eigen::MatrixXd xx(77,1) ;
  //xx <<  0.142497665407762,0.0863318274013026,0.138482846515764,-0.0794287622930922,-0.104173115885999,0.111912435729706,0.167184482618822,-0.199154522813527,-0.187028532740537,-0.219337958530823,0.0586432439663154,0.164304142708605,-0.127628173141375,-0.185743156397087,0.0170146359870672,0.0248247497410407,-0.0176064520186409,0.0662900314173315,-0.152062854424661,0.213381273867381,0.00882093196136141,0.0858712258269313,0.115846460410344,0.163804773309625,-0.0858288610582368,-0.00399305789433890,-0.134076000952463,0.104115063152663,-0.0463627028752608,-0.0403148821203501,-0.0653087472251809,0.0404520491406844,0.0123605202705737,0.0197299286730223,-0.118768448749135,0.0438521599268105,0.00199495020455419,0.0895718675401182,0.0119695010473851,0.0451003827640605,-0.0525843342003119,0.0388447166511402,0.0927363948776392,-0.115854370520480,-0.0719042898217277,-0.153937454035417,0.123556824361712,0.0557343559635552,-0.0679648347123826,-0.117088002825359,0.0653559094236643,-0.0191285284014095,0.118498435220740,-0.163430988834831,0.0879879502157222,-0.0497853011682027,-0.0462231039258255,0.160839443403243,0.0874789428964958,0.108498465915686,-0.0437769203993954,-0.249471493765010,0.125814518009108,0.0612017069902088,-0.122010392745851,-0.0476424089345723,-0.0985867064030974,0.0687648559896983,0.140669800906348,-0.141787092809230,0.0825715833931875,-0.0291119553651233,0.0948338291815862,-1.52880717511780,0.0909062089172463,1.94963262194400,0.0258756468508709;
  xx << 0.0156203936456976,0.0143186640666405,-0.0816833112923004,0.0570786688819807,0.151854545970222,0.118740542996610,0.0511264098279001,0.0146310426729426,-0.0192380824649103,0.0741743495953881,-0.0840877658300944,-0.271297721599376,-0.314068675557508,-0.0727558689895025,-0.0662984852206413,-0.00947725543095719,0.000746127915331545,0.163829393531257,0.0603221971978477,-0.0776239395123603,-0.0694855348306573,-0.0274938909471395,-0.155357926094235,-0.0535597680011939,-0.0266711992485415,0.00504806293688116,0.0657274643919639,-1.16435603182077e-06,0.0635409289044957,0.169754583948699,0.0603226017806160,-0.110149097278079,0.0540729135200865,0.207224607663680,0.0910608175866181,0.497728129314537,0.372353132488924,-0.392190174433402,0.259904910067445,0.414160890548014,0.0849155705332323,0.523516817602567,-0.0911509455029293,-0.00344508326989012,0.140932183754806,-0.326872668062816,-0.105047220101091,0.0834620982746949,-0.147065019582118,-0.217660272385917,-0.0119878144727868,0.199968871435889,0.0399368907457239,-0.112274987871691,0.0978733531925270,-0.123129471997947,0.00815230832667012,-0.286986644677726,-0.0566671438607708,0.0320360523540869,0.00174928701300472,0.0255691767577815,-5.79526417720427e-06,0.0609890723070383,0.163689418142755,-0.0525824463875476,0.0291771088764212,0.0251522731491571,-0.129547306735504,0.0280202537126400,0.626808077348408,0.521432802445549,-0.445237651993183,-1.30324421169922,0.524702115286390,2.22242524572318,0.659843543433238;
  MatrixXd a=xx.block<35,1>(0,0);
  a.resize(7,5);
  MatrixXd b=xx.block<35,1>(35,0);
  b.resize(7,5);
  MatrixXd c= xx.block<7,1>(70,0);
  c.resize(7,1);

  //q_desired = c;
  q_desired << 0,0,0,-1.5,0,2,0;
  
  double wf = 2*M_PI/10;
  int N=5;

  for (size_t i = 0; i < 7; ++i) {
    for (double l = 1;l<N; ++l) {
      q_desired[i] += a(i,l)*sin(l*wf*elapsed_time_.toSec())/(wf*l) - b(i,l)*cos(l*wf*elapsed_time_.toSec())/(wf*l);        //a(i,l)*sin(l*wf*t)/(wf*l) - b(i,l)*cos(l*wf*t)/(wf*l)
      dq_desired[i] += a(i,l)*cos(l*wf*elapsed_time_.toSec()) + b(i,l)*sin(l*wf*elapsed_time_.toSec());
      ddq_desired[i] += -a(i,l)*l*wf*sin(l*wf*elapsed_time_.toSec()) + b(i,l)*l*wf*cos(l*wf*elapsed_time_.toSec());
    }
  }
  // for (size_t i = 0; i < 7; ++i) {
  //   if (i == 4) {
  //     q_desired[i] += M_PI / 16 * (1 - cos(M_PI / 5.0 * elapsed_time_.toSec())) * 0.2;
  //     dq_desired[i] = M_PI * M_PI / 80 * sin(M_PI / 5 * elapsed_time_.toSec()) * 0.2;
  //     ddq_desired[i] = M_PI * M_PI * M_PI / 400 * cos(M_PI / 5 * elapsed_time_.toSec()) * 0.2;
  //   } else {
  //     q_desired[i] -= M_PI / 16 * (1 - cos(M_PI / 5.0 * elapsed_time_.toSec())) * 0.2;
  //     dq_desired[i] = -M_PI * M_PI / 80 * sin(M_PI / 5 * elapsed_time_.toSec()) * 0.2;
  //     ddq_desired[i] = -M_PI * M_PI * M_PI / 400 * cos(M_PI / 5 * elapsed_time_.toSec()) * 0.2;
  //     dq_desired[i] = 0;
  // ddq_desired[i] = 0;
    

  // compute error
  Matrix<double, 7, 1> q_error;
  Matrix<double, 7, 1> dq_error;
  q_error = q - q_desired;
  dq_error = dq - dq_desired;

  // gains
  k_gain << 600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0;
  d_gain << 50.0, 50.0, 50.0, 20.0, 20.0, 20.0, 10.0;
  Matrix<double, 7, 7> k_p = k_gain.asDiagonal();
  Matrix<double, 7, 7> k_d = d_gain.asDiagonal();

  // control law
  Matrix<double, 7, 1> tau_c;
  Matrix<double, 7, 1> tau_s;
  Matrix<double, 7, 1> tau_d;
  Matrix<double, 7, 1> tau_sd;
  tau_c = mass * ddq_desired + coriolis;
  tau_s = -mass * (k_p * q_error + k_d * dq_error);
  tau_d = tau_c + tau_s;
  tau_sd = saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_sd(i));
  }


  /*test for robust impedance control*/
/*   Matrix<double, 7, 7> Mass_iden = MassMatrix(q);
  Matrix<double, 7, 1> Coriolis_iden = CoriolisMatrix(q, dq) * dq;
  Matrix<double, 7, 1> Friction_iden = Friction(dq);
  Matrix<double, 7, 1> gravity_i = GravityVector(q); */
  



  // if (rate_trigger_() && CTC_out_publisher_.trylock()) {
    
  //   // Matrix<double,7,1> gravity_s=
  //   for (size_t i = 0; i < 7; i++) {
  //     /* code */
  //     CTC_out_publisher_.msg_.tau_input[i] = tau_d[i];
  //     CTC_out_publisher_.msg_.tau_sinput[i] = tau_sd[i];
  //     CTC_out_publisher_.msg_.q_error[i] = q_error[i];
  //     // CTC_out_publisher_.RealtimePublisher();
  //   }

    pos_ << elapsed_time_.toSec() << ","  << q[0] << ","<< q[1] << "," << q[2] << "," << q[3] << "," << q[4] << "," << q[5] << "," << q[6]<< std::endl;
    vel_ << elapsed_time_.toSec() << "," << dq[0] << "," << dq[1] << "," << dq[2] << "," << dq[3] << "," << dq[4] << "," << dq[5] << "," << dq[6] << std::endl;
    tau_ << elapsed_time_.toSec() << ","  <<tau_sd[0]  << "," <<tau_sd[1]  << "," << tau_sd[2] << "," << tau_sd[3] << "," << tau_sd[4] << "," << tau_sd[5] << "," << tau_sd[6] << std::endl;
    time_ << elapsed_time_.toSec() << std::endl;

}


Matrix<double, 7, 1> CTC_controller::saturateTorqueRate(const Matrix<double, 7, 1>& tau_d_calculated,const Matrix<double, 7, 1>& tau_J_d) 
{  // NOLINT (readability-identifier-naming)
  Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + max(min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

std::string CTC_controller::get_date_time() {
    auto to_string = [](const system_clock::time_point& t)->std::string {
        auto as_time_t = system_clock::to_time_t(t);
        struct tm tm;
        #if defined(WIN32) || defined(_WINDLL)
            localtime_s(&tm, &as_time_t); // win api，thread safe
        #else
            localtime_r(&as_time_t, &tm); // linux api，thread safe
        #endif
        milliseconds ms = duration_cast<milliseconds>(t.time_since_epoch());
        // char buf[128];
        // snprintf(buf, sizeof(buf), "%04d%02d%02d_%02d%02d%02d_%03ld",
        // tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, ms.count() % 1000);
        char buf[64];
        snprintf(buf, sizeof(buf), "%02d%02d%02d_%02d%02d%02d",
        tm.tm_year + 1900 - 2000, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
        return buf;
    };
    system_clock::time_point t = system_clock::now();
    return to_string(t);
}
// namespace end
}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CTC_controller, controller_interface::ControllerBase)
