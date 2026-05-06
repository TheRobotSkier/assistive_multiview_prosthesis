#ifndef MIA_HAND_DRIVER_CPP_DRIVER_HPP
#define MIA_HAND_DRIVER_CPP_DRIVER_HPP

#include <cstring>

#include <mutex>
#include <string>

#include <libserial/SerialPort.h>

namespace mia_hand_driver
{
/**
 * \brief Class implementing a C++ interface for Mia Hand devices.
 */
class CppDriver
{
public:

  /**
   * \brief Factory function for creating a new CppDriver object.
   *
   * A factory function is used for avoiding to throw exceptions in the class
   * constructor, in case of errors during object initialization.
   *
   * @return std::unique_ptr to the CppDriver object, if object creation is
   *         successful. nullptr, if object creation fails.
   */
  static std::unique_ptr<CppDriver> create();

  /* Preventing CppDriver object to be copyable, either through the copy
   * constructor and the assignment operator, to ensure the unique access to
   * them through the std::unique_ptr.
   */
  CppDriver(CppDriver&) = delete;
  CppDriver& operator=(CppDriver&) = delete;

  /**
   * \brief Class destructor.
   */
  ~CppDriver();

  /**
   * \brief Function for opening the serial port to which Mia Hand is connected.
   *
   * @param port Serial port device name (e.g., "/dev/ttyACM0"). It can be
   *             obtained, after connecting the serial cable to the PC, through
   *             \code sudo dmesg | grep tty \endcode. 
   *
   * @return \code true \endcode, if serial port opening is successful, \code
   *         false \endcode if any error occurred. In this case, information 
   *         about the error can be obtained through #get_error_msg().
   */ 
  bool open_serial_port(std::string& port);

  /**
   * \brief Function for closing the serial port to which Mia Hand is connected.
   *
   * @return \code true \endcode, if serial port closing is successful, \code
   *         false \endcode if any error occurred. In this case, information 
   *         about the error can be obtained through #get_error_msg().
   */
  bool close_serial_port();

  /**
   * \brief Function for getting the error message related to the last detected
   *        failure.
   *
   * @return Message explaining the reason for the failure.
   */
  const char* get_error_msg();

  /**
   * \brief Function for checking if Mia Hand is currently connected to the
   *        CppDriver.
   *
   * To check the connection with Mia Hand, a command with a valid format, but
   * with no effect, is sent through the serial port, verifying if the hand
   * replies with the ACK message.
   *
   * @return \code true \endcode, if Mia Hand is connected. \code false \endcode, 
   *         if the serial port is not open, or if Mia Hand is not connected to the opened serial port.
   */
  bool is_connected();

  /**
   * \brief Function for stopping all the Mia Hand motors.
   *
   * @return \code true \endcode, if motors stop was successful, \code false
   *         \endcode if not. In this case, information about the error can be
   *         obtained through #get_error_msg().
   */
  bool stop();

  /**
   * \brief Function for triggering Mia Hand emergency stop.
   *
   * When this function is called, Mia Hand motors are stopped, as for what done
   * in #stop() function. However, in this case, Mia Hand cannot be moved again
   * until #play() function is called.

   * @return \code true \endcode, if emergency stop was successful, \code false
   *         \endcode if not. In this case, information about the error can be
   *         obtained through #get_error_msg().
   */ 
  bool emergency_stop();

  /**
   * \brief Function for restoring Mia Hand normal operation after an emergency
   *        stop, issued through #emergency_stop() function.
   */
  void play();

  /**
   * \brief Function for starting a Mia Hand motor positions re-calibration.
   *
   * @return \code true \endcode, if re-calibration started successfully, \code 
   *         false \endcode if not. In this case, information about the error 
   *         can be obtained through #get_error_msg().
   */
  bool calibrate_motor_positions();

  /**
   * \brief Function for storing the current settings to Mia Hand EEPROM memory.
   *
   * @return \code true \endcode, if storing was successful, \code false 
   *         \endcode if not. In this case, information about the error 
   *         can be obtained through #get_error_msg().
   */
  bool store_current_settings();

  /**
   * \brief Function for getting Mia Hand firmware version.
   *
   * @return Firmware version message, if reading was successful. Empty message,
   *         if an error occurred. In this case, information about the error can 
   *         be obtained through #get_error_msg().
   */
  const char* get_fw_version();

  /**
   * \brief Function for getting the current motor positions.
   *
   * @param thumb_mpos Reference to variable where thumb motor position should 
   *                   be returned.
   * @param index_mpos Reference to variable where index motor position should 
   *                   be returned.
   * @param mrl_mpos Reference to variable where mrl motor position should be
   *                 returned.
   *
   * @return \code true \endcode, if motor positions reading was successful,
   *         \code false \endcode if not. In this case, information about the 
   *         error occurred can be obtained with #get_error_msg() function.
   */
  bool get_motor_positions(
    int32_t& thumb_mpos, int32_t& index_mpos, int32_t& mrl_mpos);

  /**
   * \brief Function for getting the current motor speeds.
   *
   * @param thumb_mspe Reference to variable where thumb motor speed should be
   *                   returned.
   * @param index_mspe Reference to variable where index motor speed should be
   *                   returned.
   * @param mrl_mspe Reference to variable where mrl motor speed should be
   *                 returned.
   *
   * @return \code true \endcode, if motor speeds reading was successful, \code
   *         false \endcode if not. In this case, information about the error 
   *         occurred can be obtained with #get_error_msg() function.
   */
  bool get_motor_speeds(
    int32_t& thumb_mspe, int32_t& index_mspe, int32_t& mrl_mspe);

  /**
   * \brief Function for getting the current motor currents.
   *
   * @param thumb_mcur Reference to variable where thumb motor currents should 
   *                   be returned.
   * @param index_mcur Reference to variable where index motor currents should 
   *                   be returned.
   * @param mrl_mcur Reference to variable where mrl motor currents should be
   *                 returned.
   *
   * @return \code true \endcode, if motor currents reading was successful,
   *         \code false \endcode if not. In this case, information about the 
   *         error occurred can be obtained with #get_error_msg() function.
   */
  bool get_motor_currents(
    int32_t& thumb_mcur, int32_t& index_mcur, int32_t& mrl_mcur);

  /**
   * \brief Function for getting the current joint positions in rad.
   *
   * @param thumb_jpos Reference to variable where thumb joint position should 
   *                   be returned.
   * @param index_jpos Reference to variable where index joint position should 
   *                   be returned.
   * @param mrl_jpos Reference to variable where mrl joint position should be
   *                 returned.
   *
   * @return \code true \endcode, if joint positions reading was successful,
   *         \code false \endcode if not. In this case, information about the 
   *         error occurred can be obtained with #get_error_msg() function.
   */
  bool get_joint_positions(
      double& thumb_jpos, double& index_jpos, double& mrl_jpos);

  /**
   * \brief Function for getting the current joint speeds in rad/s.
   *
   * @param thumb_jspe Reference to variable where thumb joint speed should 
   *                   be returned.
   * @param index_jspe Reference to variable where index joint speed should 
   *                   be returned.
   * @param mrl_jspe Reference to variable where mrl joint speed should be
   *                 returned.
   *
   * @return \code true \endcode, if joint speed reading was successful, \code
   *         false \endcode if not. In this case, information about the error 
   *         occurred can be obtained with #get_error_msg() function.
   */
  bool get_joint_speeds(
      double& thumb_jspe, double& index_jspe, double& mrl_jspe);

  /**
   * \brief Function for getting the current outputs of the fingertip force
   *        sensors.
   *
   * @param thumb_nfor Reference to variable where thumb normal force should be
   *                   returned.
   * @param index_nfor Reference to variable where index normal force should be
   *                   returned.
   * @param mrl_nfor Reference to variable where mrl normal force should be
   *                 returned.
   * @param thumb_tfor Reference to variable where thumb tangential force should 
   *                   be returned.
   * @param index_tfor Reference to variable where index tangential force should 
   *                   be returned.
   * @param mrl_tfor Reference to variable where mrl tangential force should be
   *                 returned.
   *
   * @return \code true \endcode, if finger forces reading was successful, \code
   *         false \endcode if not. In this case, information about the error 
   *         occurred can be obtained with #get_error_msg() function.
   */
  bool get_finger_forces(
    int32_t& thumb_nfor, int32_t& index_nfor, int32_t& mrl_nfor, 
    int32_t& thumb_tfor, int32_t& index_tfor, int32_t& mrl_tfor);

  /**
   * \brief Function for setting a target Mia Hand motor trajectory.
   *
   * @param motor Target motor number. 0: thumb, 1: index, 2: mrl.
   * @param target_pos Target position, in the interval [0, 255], or [-255, 255]
   *                   for the index motor. Values outside such range are 
   *                   saturated to the nearest limit.
   * @param traj_spe Speed (in percentage) at which the motor should reach the
   *                 target position. Such speed should be in the interval [0,
   *                 99]. Values outside such range are saturated to the nearest 
   *                 limit.
   * @return \code true \endcode if motor trajectory setting was successful, 
   *         \code false \endcode if not. In this case, information about the 
   *         error occurred can be obtained with #get_error_msg() function.
   */
  bool set_motor_trajectory(
      uint32_t motor, int32_t target_pos, int32_t traj_spe);

  /**
   * \brief Function for setting a target Mia Hand motor speed.
   *
   * @param motor Target motor number. 0: thumb, 1: index, 2: mrl.
   * @param target_spe Target speed, in the interval [-99, 99]. Values outside
   *                   such range are saturated to the nearest limit.
   * @param max_cur Maximum current (in percentage) that should be absorbed by 
   *                the motor, in the interval [0, 80]. Values outside such
   *                range are saturated to the nearest limit.
   * @return \code true \endcode if motor speed setting was successful, \code
   *         false \endcode if not. In this case, information about the error 
   *         occurred can be obtained with #get_error_msg() function.
   */
  bool set_motor_speed(
      uint32_t motor, int32_t target_spe, int32_t max_cur);

  /**
   * \brief Function for setting a target Mia Hand joint trajectory.
   *
   * @param joint Target joint number. 0: thumb, 1: index, 2: mrl.
   * @param target_pos Target position, in rad. Positions values are rounded at 
   *                   the second decimal digit (e.g., if a position of 1.235 rad 
   *                   is specified, the target position set will be 1.24 rad).
   * @param traj_spe Speed (in percentage) at which the joint should reach the
   *                 target position. Such speed should be in the interval [0,
   *                 99]. Values outside such range are saturated to the nearest 
   *                 limit.
   * @return \code true \endcode if joint trajectory setting was successful, 
   *         \code false \endcode if not. In this case, information about the 
   *         error occurred can be obtained with #get_error_msg() function.
   */
  bool set_joint_trajectory(
      uint32_t joint, float target_pos, int32_t traj_spe);

  /**
   * \brief Function for setting a target Mia Hand joint speed.
   *
   * @param joint Target joint number. 0: thumb, 1: index, 2: mrl.
   * @param target_spe Target speed, in rad/s. Speed values are rounded at the
   *                   first decimal digit (e.g., if a speed of 1.25 rad/s is
   *                   specified, the target speed set will be 1.3 rad/s).
   * @param max_cur Maximum current (in percentage) that should be absorbed by 
   *                the related motor, in the interval [0, 80]. Values outside 
   *                such range are saturated to the nearest limit.
   * @return \code true \endcode if joint speed setting was successful, \code
   *         false \endcode if not. In this case, information about the error 
   *         occurred can be obtained with #get_error_msg() function.
   */
  bool set_joint_speed(
      uint32_t joint, float target_spe, int32_t max_cur);

  /**
   * \brief Function for reading the open and close motor positions of a certain 
   *        grasp. 
   *
   * @param grasp_id Target grasp ID. 'C': cylindrical, 'P': pinch, 'L':
   *                 lateral, 'U': point-up, 'D': point-down, 'S': spherical, 
   *                 'T': tridigital, 'R': relax.
   * @param motor Target motor number. 0: thumb, 1: index, 2: mrl.
   * @param open_mpos Variable where grasp OPEN position should be returned.
   * @param close_mpos Variable where grasp CLOSE position should be returned.
   *
   * @return \code true \endcode, if grasp references reading was successful, 
   *         \code false \endcode if not. In this case, information about the 
   *         error occurred can be obtained with #get_error_msg() function.
   */
  bool get_grasp_refs(
      char grasp_id, uint32_t motor, int32_t& open_mpos, int32_t& close_mpos);

  /**
   * \brief Function for setting the open and close motor positions of a certain 
   *        grasp. 
   *
   * @param grasp_id Target grasp ID. 'C': cylindrical, 'P': pinch, 'L':
   *                 lateral, 'U': point-up, 'D': point-down, 'S': spherical, 
   *                 'T': tridigital, 'R': relax.
   * @param motor Target motor number. 0: thumb, 1: index, 2: mrl.
   * @param open_mpos OPEN position.
   * @param close_mpos CLOSE position.
   *
   * @return \code true \endcode, if grasp references setting was successful, 
   *         \code false \endcode if not. In this case, information about the 
   *         error occurred can be obtained with #get_error_msg() function.
   */
  bool set_grasp_refs(
      char grasp_id, uint32_t motor, int32_t open_mpos, int32_t close_mpos);

  /**
   * \brief Function for executing a grasp.
   *
   * @param grasp_id Target grasp ID. 'C': cylindrical, 'P': pinch, 'L':
   *                 lateral, 'U': point-up, 'D': point-down, 'S': spherical, 
   *                 'T': tridigital, 'R': relax.
   * @param close_percent Grasp closure percentage.
   * @param spe_percent Grasp execution speed/force percentage.
   *
   * @return \code true \endcode, if grasp references setting was successful, 
   *         \code false \endcode if not. In this case, information about the 
   *         error occurred can be obtained with #get_error_msg() function.
   */
  bool execute_grasp(
      char grasp_id, int32_t close_percent, int32_t spe_for_percent);

private:

  /**
   * \brief Function for initializing a CppDriver object.
   *
   * This function is necessary for accessing all the non-static members through
   * the static factory function #create().
   *
   * @return true, if object initialization was successful, false if not. 
   */ 
  bool init();

  /**
   * \brief Class constructor.
   *
   * The class constructor is private, to allow instantiations of new CppDriver
   * objects only through #create() factory function.
   */
  CppDriver();

  /**
   * \brief Function for sending a command to Mia Hand through the serial port.
   *
   * @return true, if the command is successfully received by the hand (i.e.,
   *         if the hand sends back an ACK message). false, if no ACK message is
   *         received from the hand before a pre-defined timeout.
   */
  bool send_command(const std::string& cmd);

  LibSerial::SerialPort serial_port_;  //!< Mia Hand serial port object.

  std::string rx_msg_;  //!< Message received from the serial port.

  bool emergency_stop_on_;  //!< Flag for signaling emergency stop on.

  char err_msg_[128];  //!< Error message, filled after any occurred error.

  std::mutex data_mtx_;  //!< Mutex for functions thread-safety.
};
}  // namespace

#endif  // MIA_HAND_DRIVER_CPP_DRIVER_HPP

