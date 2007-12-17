/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *  
 **/
/* Desc: Static plugin automagic registration
 ** Author: Jordi Polo 
 ** Date: 13 December 2007
 **/

#ifndef STATIC_PLUGIN_REGISTER_H
#define STATIC_PLUGIN_REGISTER_H
/*
 * Each staticPlugin fills this structure with their own initilization funtion
 * Done in a structure that it is initialized in a .hh and thus called when the
 * program runs (that's why they are static)
 * Usually that means that they just register themself in their proper factory.
 * */
struct StaticPluginRegister
{
  StaticPluginRegister(void (*initFunc)())
  {
    (*initFunc)();
  }
};
#endif 
