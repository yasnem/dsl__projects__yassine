/* 
**  Xbox/Xbox360 USB Gamepad Userspace Driver
**  Copyright (C) 2008 Ingo Ruhnke <grumbel@gmx.de>
**
**  This program is free software: you can redistribute it and/or modify
**  it under the terms of the GNU General Public License as published by
**  the Free Software Foundation, either version 3 of the License, or
**  (at your option) any later version.
**
**  This program is distributed in the hope that it will be useful,
**  but WITHOUT ANY WARRANTY; without even the implied warranty of
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
**  GNU General Public License for more details.
**
**  You should have received a copy of the GNU General Public License
**  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef HEADER_UINPUT_DRIVER_HPP
#define HEADER_UINPUT_DRIVER_HPP

#include <linux/uinput.h>
#include "control.hpp"

/** */
class UInputDriver : public Control
{
private:
  uinput_user_dev user_dev;
  bool abs_bit;
  bool rel_bit;
  bool key_bit;
  int  fd;

public:
  UInputDriver(const std::string& name);
  ~UInputDriver();

  std::string get_name() const { return "Uinput"; }

  /*@{*/
  /** Create an absolute axis */
  void add_abs(uint16_t code, int min, int max);

  /** Create an button */
  void add_btn(uint16_t code);

  /** Create a relative axis (mice) */
  void add_rel(uint16_t code);
  
  /** Finish*/
  void finish();
  /*@}*/

private:
  /** Called when one of the ports of this Control changes */
  /*@{*/
  void on_abs(AbsPortOut* port, uint16_t code);
  void on_rel(RelPortOut* port, uint16_t code);
  void on_btn(BtnPortOut* port, uint16_t code);
  /*@}*/
private:
  UInputDriver (const UInputDriver&);
  UInputDriver& operator= (const UInputDriver&);
};

#endif

/* EOF */
