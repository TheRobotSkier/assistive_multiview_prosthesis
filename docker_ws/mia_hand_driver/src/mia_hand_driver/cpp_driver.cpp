#include "mia_hand_driver/cpp_driver.hpp"

#include <charconv>

namespace mia_hand_driver
{
std::unique_ptr<CppDriver> CppDriver::create()
{
  std::unique_ptr<CppDriver> ptr = std::unique_ptr<CppDriver>(new CppDriver());

  return ptr;
}

CppDriver::~CppDriver()
{
  try
  {
    if (serial_port_.IsOpen())
    {
      serial_port_.Close();
    }
  }
  catch (...)
  {
    // Avoiding any exception to be thrown in the destructor.
  }
}

bool CppDriver::open_serial_port(std::string& port)
{
  bool success = true;

  if (serial_port_.IsOpen())
  {
    serial_port_.Close();
  }

  try
  {
    serial_port_.Open(port);

    serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    serial_port_.FlushIOBuffers();
  }
  catch (std::exception& err)
  {
    success = false;
    strcpy(err_msg_, err.what());
  }

  return success;
}

bool CppDriver::close_serial_port()
{
  bool success = true;

  /* Closing serial port, if still open.
   */
  try
  {
    if (serial_port_.IsOpen())
    {
      serial_port_.Close();
    }
  }
  catch (std::exception& err)
  {
    success = false;
    strcpy(err_msg_, err.what());
  }

  return success;
}

const char* CppDriver::get_error_msg()
{
  return err_msg_;
}

bool CppDriver::is_connected()
{
  return send_command("@MiaHandCommand.*\r");
}

bool CppDriver::stop()
{
  return send_command("@AS.............*\r");
}

bool CppDriver::emergency_stop()
{
  if (stop())
  {
    emergency_stop_on_ = true;
  }
  else
  {
    emergency_stop_on_ = false;
  }

  return emergency_stop_on_;
}

void CppDriver::play()
{
  emergency_stop_on_ = false;

  return;
}

bool CppDriver::calibrate_motor_positions()
{
  return send_command("@AF.............*\r");
}

bool CppDriver::store_current_settings()
{
  return send_command("@ES.............*\r");
}

const char* CppDriver::get_fw_version()
{
  bool success = send_command("@SR.............*\r");

  if (success)
  {
    /* Reading firmware message.
     */
    try 
    {
      serial_port_.ReadLine(rx_msg_, '\n', 20);
    }
    catch (std::exception& err)
    {
      success = false;
      strcpy(err_msg_, 
        "Timeout occurred before receiving firmware version from Mia Hand.");
      rx_msg_[0] = '\0';
      serial_port_.FlushInputBuffer();
    }
  }

  if (success)
  {
    /* Checking firmware message format.
     */
    std::size_t fw_version_msg_len = rx_msg_.length();

    if (('F'!= rx_msg_[0]) || 
        ((10 != fw_version_msg_len) && (28 != fw_version_msg_len)))
    {
      success = false;
      strcpy(err_msg_, "Invalid firmware message format.");
      rx_msg_[0] = '\0';
    }
  }

  return rx_msg_.c_str();
}

bool CppDriver::get_motor_positions(
  int32_t& thumb_mpos, int32_t& index_mpos, int32_t& mrl_mpos)
{
  bool success = send_command("@ADPo...........*\r");

  if (success)
  {
    /* Reading motor positions message.
     */
    try 
    {
      serial_port_.Read(rx_msg_, 31, 20);
    }
    catch (std::exception& err)
    {
      success = false;
      strcpy(err_msg_, 
        "Timeout occurred before receiving Mia Hand motor positions.");
      serial_port_.FlushInputBuffer();
    }
  }

  if (success)
  {
    /* Parsing message.
     */
    if ('e' == rx_msg_[0])
    {
      thumb_mpos = (rx_msg_[9] - 48) * 100 + (rx_msg_[10] - 48) * 10 
        + rx_msg_[11] - 48;

      if ('-' == rx_msg_[6])
      {
        thumb_mpos = -thumb_mpos;
      }

      index_mpos = (rx_msg_[27] - 48) * 100 + (rx_msg_[28] - 48) * 10 
        + rx_msg_[29] - 48;

      if ('-' == rx_msg_[24])
      {
        index_mpos = -index_mpos;
      }

      mrl_mpos = (rx_msg_[18] - 48) * 100 + (rx_msg_[19] - 48) * 10 
        + rx_msg_[20] - 48;

      if ('-' == rx_msg_[15])
      {
        mrl_mpos = -mrl_mpos;
      }
    }
    else
    {
      success = false;
      strcpy(err_msg_, "Invalid motor positions message format.");
    }
  }

  return success;
}

bool CppDriver::get_motor_speeds(
  int32_t& thumb_mspe, int32_t& index_mspe, int32_t& mrl_mspe)
{
  bool success = send_command("@ADSo...........*\r");

  if (success)
  {
    /* Reading motor speeds message.
     */
    try 
    {
      serial_port_.Read(rx_msg_, 31, 20);
    }
    catch (std::exception& err)
    {
      success = false;
      strcpy(err_msg_, 
        "Timeout occurred before receiving Mia Hand motor speeds.");
      serial_port_.FlushInputBuffer();
    }
  }

  if (success)
  {
    /* Parsing message.
     */
    if ('s' == rx_msg_[0])
    {
      thumb_mspe = (rx_msg_[10] - 48) * 10 + rx_msg_[11] - 48;

      if ('-' == rx_msg_[6])
      {
        thumb_mspe = -thumb_mspe;
      }

      index_mspe = (rx_msg_[28] - 48) * 10 + rx_msg_[29] - 48;

      if ('-' == rx_msg_[24])
      {
        index_mspe = -index_mspe;
      }

      mrl_mspe = (rx_msg_[19] - 48) * 10 + rx_msg_[20] - 48;

      if ('-' == rx_msg_[15])
      {
        mrl_mspe = -mrl_mspe;
      }
    }
    else
    {
      success = false;
      strcpy(err_msg_, "Invalid motor speeds message format.");
    }
  }

  return success;
}

bool CppDriver::get_motor_currents(
  int32_t& thumb_mcur, int32_t& index_mcur, int32_t& mrl_mcur)
{
  bool success = send_command("@ADCo...........*\r");

  if (success)
  {
    /* Reading motor currents message.
     */
    try 
    {
      serial_port_.Read(rx_msg_, 31, 20);
    }
    catch (std::exception& err)
    {
      success = false;
      strcpy(err_msg_, 
        "Timeout occurred before receiving Mia Hand motor currents.");
      serial_port_.FlushInputBuffer();
    }
  }

  if (success)
  {
    /* Parsing message.
     */
    if ('c' == rx_msg_[0])
    {
      thumb_mcur = (rx_msg_[8] - 48) * 1000 + (rx_msg_[9] - 48) * 100 
        + (rx_msg_[10] - 48) * 10 + rx_msg_[11] - 48;

      if ('-' == rx_msg_[6])
      {
        thumb_mcur = -thumb_mcur;
      }

      index_mcur = (rx_msg_[26] - 48) * 1000 + (rx_msg_[27] - 48) * 100 
        + (rx_msg_[28] - 48) * 10 + rx_msg_[29] - 48;

      if ('-' == rx_msg_[24])
      {
        index_mcur = -index_mcur;
      }

      mrl_mcur = (rx_msg_[17] - 48) * 1000 + (rx_msg_[18] - 48) * 100 
        + (rx_msg_[19] - 48) * 10 + rx_msg_[20] - 48;

      if ('-' == rx_msg_[15])
      {
        mrl_mcur = -mrl_mcur;
      }
    }
    else
    {
      success = false;
      strcpy(err_msg_, "Invalid motor currents message format.");
    }
  }

  return success;
}

bool CppDriver::get_joint_positions(
    double& thumb_jpos, double& index_jpos, double& mrl_jpos)
{
  bool success = send_command("@ADJo...........*\r");

  if (success)
  {
    /* Reading joint positions message.
     */
    try 
    {
      serial_port_.Read(rx_msg_, 15, 20);
    }
    catch (std::exception& err)
    {
      success = false;
      strcpy(err_msg_, 
        "Timeout occurred before receiving Mia Hand joint positions.");
      serial_port_.FlushInputBuffer();
    }
  }

  if (success)
  {
    /* Parsing message.
     */
    if (('J' == rx_msg_[0]) && ('P' == rx_msg_[1]))
    {
      thumb_jpos = ((rx_msg_[3] - 48) * 100 + (rx_msg_[4] - 48) * 10 
          + rx_msg_[5] - 48) / 100.0f;

      if ('-' == rx_msg_[2])
      {
        thumb_jpos = -thumb_jpos;
      }

      index_jpos = ((rx_msg_[11] - 48) * 100 + (rx_msg_[12] - 48) * 10 
          + rx_msg_[13] - 48) / 100.0f;

      if ('-' == rx_msg_[10])
      {
        index_jpos = -index_jpos;
      }

      mrl_jpos = ((rx_msg_[7] - 48) * 100 + (rx_msg_[8] - 48) * 10 
          + rx_msg_[9] - 48) / 100.0f;

      if ('-' == rx_msg_[6])
      {
        mrl_jpos = -mrl_jpos;
      }
    }
    else
    {
      success = false;
      strcpy(err_msg_, "Invalid joint positions message format.");
    }
  }

  return success;
}

bool CppDriver::get_joint_speeds(
    double& thumb_jspe, double& index_jspe, double& mrl_jspe)
{
  bool success = send_command("@ADjo...........*\r");

  if (success)
  {
    /* Reading joint speeds message.
     */
    try 
    {
      serial_port_.Read(rx_msg_, 12, 20);
    }
    catch (std::exception& err)
    {
      success = false;
      strcpy(err_msg_, 
        "Timeout occurred before receiving Mia Hand joint speeds.");
      serial_port_.FlushInputBuffer();
    }
  }

  if (success)
  {
    /* Parsing message.
     */
    if (('J' == rx_msg_[0]) && ('S' == rx_msg_[1]))
    {
      thumb_jspe = ((rx_msg_[3] - 48) * 10 + rx_msg_[4] - 48) / 10.0f;

      if ('-' == rx_msg_[2])
      {
        thumb_jspe = -thumb_jspe;
      }

      index_jspe = ((rx_msg_[9] - 48) * 10 + rx_msg_[10] - 48) / 10.0f;

      if ('-' == rx_msg_[8])
      {
        index_jspe = -index_jspe;
      }

      mrl_jspe = ((rx_msg_[6] - 48) * 10 + rx_msg_[7] - 48) / 10.0f;

      if ('-' == rx_msg_[5])
      {
        mrl_jspe = -mrl_jspe;
      }
    }
    else
    {
      success = false;
      strcpy(err_msg_, "Invalid joint speeds message format.");
    }
  }

  return success;
}

bool CppDriver::get_finger_forces(
    int32_t& thumb_nfor, int32_t& index_nfor, int32_t& mrl_nfor, 
    int32_t& thumb_tfor, int32_t& index_tfor, int32_t& mrl_tfor)
{
  bool success = send_command("@ADAo...........*\r");

  if (success)
  {
    /* Reading finger forces message.
     */
    try 
    {
      serial_port_.Read(rx_msg_, 76, 20);
    }
    catch (std::exception& err)
    {
      success = false;
      strcpy(err_msg_, 
        "Timeout occurred before receiving Mia Hand finger forces.");
      serial_port_.FlushInputBuffer();
    }
  }

  if (success)
  {
    /* Parsing message.
     */
    if ('a' == rx_msg_[0])
    {
      thumb_nfor = (rx_msg_[44] - 48) * 1000 + (rx_msg_[45] - 48) * 100 
        + (rx_msg_[46] - 48) * 10 + rx_msg_[47] - 48;

      if ('-' == rx_msg_[42])
      {
        thumb_nfor = -thumb_nfor;
      }

      index_nfor = (rx_msg_[17] - 48) * 1000 + (rx_msg_[18] - 48) * 100 
        + (rx_msg_[19] - 48) * 10 + rx_msg_[20] - 48;

      if ('-' == rx_msg_[15])
      {
        index_nfor = -index_nfor;
      }

      mrl_nfor = (rx_msg_[53] - 48) * 1000 + (rx_msg_[54] - 48) * 100 
        + (rx_msg_[55] - 48) * 10 + rx_msg_[56] - 48;

      if ('-' == rx_msg_[51])
      {
        mrl_nfor = -mrl_nfor;
      }

      thumb_tfor = (rx_msg_[35] - 48) * 1000 + (rx_msg_[36] - 48) * 100 
        + (rx_msg_[37] - 48) * 10 + rx_msg_[38] - 48;

      if ('-' == rx_msg_[33])
      {
        thumb_tfor = -thumb_tfor;
      }

      index_tfor = (rx_msg_[26] - 48) * 1000 + (rx_msg_[27] - 48) * 100 
        + (rx_msg_[28] - 48) * 10 + rx_msg_[29] - 48;

      if ('-' == rx_msg_[24])
      {
        index_tfor = -index_tfor;
      }

      /* Parsing mrl tangential force.
       */
      mrl_tfor = (rx_msg_[8] - 48) * 1000 + (rx_msg_[9] - 48) * 100 
        + (rx_msg_[10] - 48) * 10 + rx_msg_[11] - 48;

      if ('-' == rx_msg_[6])
      {
        mrl_tfor = -mrl_tfor;
      }
    }
    else
    {
      success = false;
      strcpy(err_msg_, "Invalid finger forces message format.");
    }
  }

  return success;
}

bool CppDriver::set_motor_trajectory(
    uint32_t motor, int32_t target_pos, int32_t traj_spe)
{
  bool success = true;

  if (!emergency_stop_on_)
  {
    char cmd[19] = "@xP+000000......*\r";

    /* Mapping motor number into ASCII ID for the command.
     */
    switch (motor)
    {
      case 0:

        cmd[1] = '1';

        break;

      case 1:

        cmd[1] = '3';

        break;

      case 2:

        cmd[1] = '2';

        break;

      default:

        success = false;
        strcpy(err_msg_, "Invalid motor number.");

        break;
    }

    if (success)
    {
      /* Converting target position to ASCII format.
       */
      if (target_pos < 0)
      {
        if (1 != motor)  // Target motor is thumb or mrl one
        {
          target_pos = 0;  // Saturating target position.
        }
        else  // Target motor is index one
        {
          cmd[3] = '-';
          target_pos = -target_pos;
        }
      }

      if (target_pos > 99)
      {
        if (target_pos < 255)
        {
          std::to_chars(&cmd[5], &cmd[8], target_pos);
        }
        else
        {
          memcpy(&cmd[5], "255", 3);
        }
      }
      else if (target_pos > 9)
      {
        std::to_chars(&cmd[6], &cmd[8], target_pos);
      }
      else
      {
        std::to_chars(&cmd[7], &cmd[8], target_pos);
      }

      /* Converting speed percentage to ASCII format.
       */
      if (traj_spe > 99)
      {
        memcpy(&cmd[8], "99", 2);
      }
      else if (traj_spe > 9)
      {
        std::to_chars(&cmd[8], &cmd[10], traj_spe);
      }
      else if (traj_spe > 0)
      {
        std::to_chars(&cmd[9], &cmd[10], traj_spe);
      }
      else
      {
      }

      /* Sending command to the hand.
       */
      success = send_command(cmd);
    }
  }
  else
  {
    success = false;
    strcpy(err_msg_, "Emergency stop on.");
  }

  return success;
}

bool CppDriver::set_motor_speed(
    uint32_t motor, int32_t target_spe, int32_t max_cur)
{
  bool success = true;

  if (!emergency_stop_on_)
  {
    char cmd[19] = "@xS+....0000....*\r";

    /* Mapping motor number into ASCII ID for the command.
     */
    switch (motor)
    {
      case 0:

        cmd[1] = '1';

        break;

      case 1:

        cmd[1] = '3';

        break;

      case 2:

        cmd[1] = '2';

        break;

      default:

        success = false;
        strcpy(err_msg_, "Invalid motor number.");

        break;
    }

    if (success)
    {
      /* Converting target speed to ASCII format.
       */
      if (target_spe < 0)
      {
        cmd[3] = '-';
        target_spe = -target_spe;
      }

      if (target_spe > 99)
      {
        memcpy(&cmd[8], "99", 2);  // Saturating target speed.
      }
      else if (target_spe > 9)
      {
        std::to_chars(&cmd[8], &cmd[10], target_spe);
      }
      else
      {
        std::to_chars(&cmd[9], &cmd[10], target_spe);
      }

      /* Converting maximum current to ASCII format.
       */
      if (max_cur > 80)
      {
        memcpy(&cmd[10], "80", 2);  // Saturating maximum current.
      }
      else if (max_cur > 9)
      {
        std::to_chars(&cmd[10], &cmd[12], max_cur);
      }
      else if (max_cur > 0)
      {
        std::to_chars(&cmd[11], &cmd[12], max_cur);
      }
      else
      {
      }

      /* Sending command to the hand.
       */
      success = send_command(cmd);
    }
  }
  else
  {
    success = false;
    strcpy(err_msg_, "Emergency stop on.");
  }

  return success;
}

bool CppDriver::set_joint_trajectory(
    uint32_t joint, float target_pos, int32_t traj_spe)
{
  bool success = true;

  if (!emergency_stop_on_)
  {
    char cmd[19] = "@JxP+00000......*\r";

    /* Mapping joint number into ASCII ID for the command.
     */
    switch (joint)
    {
      case 0:

        cmd[2] = '1';

        break;

      case 1:

        cmd[2] = '3';

        break;

      case 2:

        cmd[2] = '2';

        break;

      default:

        success = false;
        strcpy(err_msg_, "Invalid joint number.");

        break;
    }

    if (success)
    {
      /* Converting target position to ASCII format.
       */
      if (target_pos < 0.00f)
      {
        if (1 != joint)  // Target motor is thumb or mrl one
        {
          target_pos = 0.00f;  // Saturating target position.
        }
        else  // Target motor is index one
        {
          cmd[4] = '-';
          target_pos = -target_pos;
        }
      }

      if (target_pos < 3.00f)
      {
        int32_t target_pos_int = target_pos * 100.00f;

        if (target_pos_int > 99)
        {
          std::to_chars(&cmd[5], &cmd[8], target_pos_int);
        }
        else if (target_pos_int > 9)
        {
          std::to_chars(&cmd[6], &cmd[8], target_pos_int);
        }
        else
        {
          std::to_chars(&cmd[7], &cmd[8], target_pos_int);
        }
      }
      else
      {
        memcpy(&cmd[5], "300", 3);  // Saturating target position.
      }

      /* Converting speed percentage to ASCII format.
       */
      if (traj_spe > 99)
      {
        memcpy(&cmd[8], "99", 2);
      }
      else if (traj_spe > 9)
      {
        std::to_chars(&cmd[8], &cmd[10], traj_spe);
      }
      else if (traj_spe > 0)
      {
        std::to_chars(&cmd[9], &cmd[10], traj_spe);
      }
      else
      {
      }

      /* Sending command to the hand.
       */
      success = send_command(cmd);
    }
  }
  else
  {
    success = false;
    strcpy(err_msg_, "Emergency stop on.");
  }

  return success;
}

bool CppDriver::set_joint_speed(
    uint32_t joint, float target_spe, int32_t max_cur)
{
  bool success = true;

  if (!emergency_stop_on_)
  {
    char cmd[19] = "@JxS+0000.......*\r";

    /* Mapping joint number into ASCII ID for the command.
     */
    switch (joint)
    {
      case 0:

        cmd[2] = '1';

        break;

      case 1:

        cmd[2] = '3';

        break;

      case 2:

        cmd[2] = '2';

        break;

      default:

        success = false;
        strcpy(err_msg_, "Invalid joint number.");

        break;
    }

    if (success)
    {
      /* Converting target speed to ASCII format.
       */
      if (target_spe < 0.0f)
      {
        cmd[4] = '-';
        target_spe = -target_spe;
      }

      if (target_spe < 9.9f)
      {
        int32_t target_spe_int = target_spe * 10.0f;

        if (target_spe_int > 9)
        {
          std::to_chars(&cmd[5], &cmd[7], target_spe_int);
        }
        else
        {
          std::to_chars(&cmd[6], &cmd[7], target_spe_int);
        }
      }
      else
      {
        memcpy(&cmd[5], "99", 2);  // Saturating target speed.
      }

      /* Converting maximum current to ASCII format.
       */
      if (max_cur > 80)
      {
        memcpy(&cmd[7], "80", 2);  // Saturating maximum current.
      }
      else if (max_cur > 9)
      {
        std::to_chars(&cmd[7], &cmd[9], max_cur);
      }
      else if (max_cur > 0)
      {
        std::to_chars(&cmd[8], &cmd[9], max_cur);
      }
      else
      {
      }

      /* Sending command to the hand.
       */
      success = send_command(cmd);
    }
  }
  else
  {
    success = false;
    strcpy(err_msg_, "Emergency stop on.");
  }

  return success;
}

bool CppDriver::get_grasp_refs(
    char grasp_id, uint32_t motor, int32_t& open_mpos, int32_t& close_mpos)
{
  bool success = true;
  char cmd[19] = "@xgx............*\r";

  /* Mapping joint number into ASCII ID for the command.
   */
  switch (motor)
  {
    case 0:

      cmd[1] = '1';

      break;

    case 1:

      cmd[1] = '3';

      break;

    case 2:

      cmd[1] = '2';

      break;

    default:

      success = false;
      strcpy(err_msg_, "Invalid motor number.");

      break;
  }

  if (success)
  {
    /* Checking grasp ID validity.
     */
    if (
        ('C' != grasp_id) && ('P' != grasp_id) && ('L' != grasp_id) && 
        ('U' != grasp_id) && ('D' != grasp_id) && ('S' != grasp_id) &&
        ('T' != grasp_id) && ('R' != grasp_id))
    {
      success = false;
      strcpy(err_msg_, "Invalid grasp ID.");
    }
    else
    {
      cmd[3] = grasp_id;
    }
  }

  if (success)
  {
    success = send_command(cmd);
  }

  if (success)
  {
    /* Reading grasp references message.
     */
    try 
    {
      serial_port_.Read(rx_msg_, 29, 20);
    }
    catch (std::exception& err)
    {
      success = false;
      strcpy(err_msg_, 
        "Timeout occurred before receiving Mia Hand grasp references.");
      serial_port_.FlushInputBuffer();
    }
  }

  if (success)
  {
    /* Parsing message.
     */
    if ('G' == rx_msg_[0])
    {
      open_mpos = (rx_msg_[11] - 48) * 100 + (rx_msg_[12] - 48) * 10
        + rx_msg_[13] - 48;

      if ('-' == rx_msg_[10])
      {
        open_mpos = -open_mpos;
      }

      close_mpos = (rx_msg_[18] - 48) * 100 + (rx_msg_[19] - 48) * 10
        + rx_msg_[20] - 48;

      if ('-' == rx_msg_[17])
      {
        close_mpos = -close_mpos;
      }
    }
    else
    {
      success = false;
      strcpy(err_msg_, "Invalid grasp references message format.");
    }
  }

  return success;
}

bool CppDriver::set_grasp_refs(
    char grasp_id, uint32_t motor, int32_t open_mpos, int32_t close_mpos)
{
  bool success = true;
  char cmd[19] = "@xGx+000+000....*\r";

  /* Mapping joint number into ASCII ID for the command.
   */
  switch (motor)
  {
    case 0:

      cmd[1] = '1';

      break;

    case 1:

      cmd[1] = '3';

      break;

    case 2:

      cmd[1] = '2';

      break;

    default:

      success = false;
      strcpy(err_msg_, "Invalid motor number.");

      break;
  }

  if (success)
  {
    /* Checking grasp ID validity.
     */
    if (
        ('C' != grasp_id) && ('P' != grasp_id) && ('L' != grasp_id) && 
        ('U' != grasp_id) && ('D' != grasp_id) && ('S' != grasp_id) &&
        ('T' != grasp_id) && ('R' != grasp_id))
    {
      success = false;
      strcpy(err_msg_, "Invalid grasp ID.");
    }
    else
    {
      cmd[3] = grasp_id;
    }
  }

  if (success)
  {
    /* Converting OPEN position to ASCII format.
     */
    if (open_mpos < 0)
    {
      if (1 != motor)  // Target motor is thumb or mrl one
      {
        open_mpos = 0;  // Saturating OPEN position.
      }
      else  // Target motor is index one
      {
        cmd[4] = '-';
        open_mpos = -open_mpos;
      }
    }

    if (open_mpos > 99)
    {
      if (open_mpos < 255)
      {
        std::to_chars(&cmd[5], &cmd[8], open_mpos);
      }
      else
      {
        memcpy(&cmd[5], "255", 3);
      }
    }
    else if (open_mpos > 9)
    {
      std::to_chars(&cmd[6], &cmd[8], open_mpos);
    }
    else
    {
      std::to_chars(&cmd[7], &cmd[8], open_mpos);
    }

    /* Converting CLOSE position to ASCII format.
     */
    if (close_mpos < 0)
    {
      if (1 != motor)  // Target motor is thumb or mrl one
      {
        close_mpos = 0;  // Saturating CLOSE position.
      }
      else  // Target motor is index one
      {
        cmd[8] = '-';
        close_mpos = -close_mpos;
      }
    }

    if (close_mpos > 99)
    {
      if (close_mpos < 255)
      {
        std::to_chars(&cmd[9], &cmd[12], close_mpos);
      }
      else
      {
        memcpy(&cmd[9], "255", 3);
      }
    }
    else if (close_mpos > 9)
    {
      std::to_chars(&cmd[10], &cmd[12], close_mpos);
    }
    else
    {
      std::to_chars(&cmd[11], &cmd[12], close_mpos);
    }

    success = send_command(cmd);
  }

  return success;
}

bool CppDriver::execute_grasp(
    char grasp_id, int32_t close_percent, int32_t spe_for_percent)
{
  bool success = true;
  char cmd[19] = "@AGex000000.....*\r";

  /* Checking grasp ID validity.
   */
  if (
      ('C' != grasp_id) && ('P' != grasp_id) && ('L' != grasp_id) && 
      ('U' != grasp_id) && ('D' != grasp_id) && ('S' != grasp_id) &&
      ('T' != grasp_id) && ('R' != grasp_id))
  {
    success = false;
    strcpy(err_msg_, "Invalid grasp ID.");
  }
  else
  {
    cmd[4] = grasp_id;
  }

  if (success)
  {
    /* Converting closure percentage in ASCII format.
     */
    if (close_percent > 99)
    {
      cmd[5] = '1';  // Saturating closure percentage to 100.
    }
    else if (close_percent > 9)
    {
      std::to_chars(&cmd[6], &cmd[8], close_percent);
    }
    else if (close_percent > 0)
    {
      std::to_chars(&cmd[7], &cmd[8], close_percent);
    }
    else
    {
    }

    /* Converting speed/force percentage in ASCII format.
     */
    if (spe_for_percent > 99)
    {
      cmd[8] = '1';  // Saturating speed/force percentage to 100.
    }
    else if (spe_for_percent > 9)
    {
      std::to_chars(&cmd[9], &cmd[11], spe_for_percent);
    }
    else if (spe_for_percent > 0)
    {
      std::to_chars(&cmd[10], &cmd[11], spe_for_percent);
    }
    else
    {
    }

    success = send_command(cmd);
  }

  return success;
}

bool CppDriver::init()
{
  bool success = true;

  return success;
}

CppDriver::CppDriver():
  err_msg_("")
{
}

bool CppDriver::send_command(const std::string& cmd)
{
  bool success = true;

  /* If command format is valid, sending it to the hand.
   */
  if ((18 == cmd.length()) && ('@' == cmd[0]) &&
      ('*' == cmd[16]) && ('\r' == cmd[17]))
  {
    try
    {
      serial_port_.Write(cmd);
      serial_port_.DrainWriteBuffer();
    }
    catch (std::exception& err)
    {
      success = false;
      strcpy(err_msg_, err.what());
    }
  }
  else
  {
    success = false;
    strcpy(err_msg_, "Invalid command format.");
  }

  if (success)
  {
    /* Reading ACK.
     */
    try 
    {
      serial_port_.Read(rx_msg_, 17, 20);
    }
    catch (std::exception& err)
    {
      success = false;
      strcpy(err_msg_, 
          "Timeout occurred before receiving command acknowledge from Mia Hand.");
      serial_port_.FlushInputBuffer();
    }
  }

  if (success)
  {
    /* Checking correct ACK format.
     */
    if (('<' != rx_msg_[0]) || (0 != cmd.compare(1, 14, rx_msg_, 1, 14)))
    {
      success = false;
      strcpy(err_msg_, "Invalid ACK received back from Mia Hand.");
      serial_port_.FlushInputBuffer();
    }
  }

  return success;
}
}  // namespace
