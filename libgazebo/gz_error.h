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
 */

/*
 * Desc: Error handling
 * Author: Andrew Howard
 * Date: 21 Apr 2003
 * CVS: $Id: gz_error.h,v 1.6.2.2 2006/12/16 22:43:22 natepak Exp $
 */

#ifndef GZ_ERROR_H
#define GZ_ERROR_H

#include <stdio.h>


/** @brief @internal Function for print and logging errors.  Do not
    call this function directly; use the macros below.
*/
void gz_error_print(const char *type, int level, const char *file, int line, const char *fmt, ...);


/** Error macros */
#define GZ_ERROR(m) \
  gz_error_print("error", 0, __FILE__, __LINE__, m)
#define GZ_ERROR1(m, a) \
  gz_error_print("error", 0, __FILE__, __LINE__, m, a)
#define GZ_ERROR2(m, a, b) \
  gz_error_print("error", 0, __FILE__, __LINE__, m, a, b)

/** Diagnostic macros */
#define GZ_MSG(level, m) \
  gz_error_print("msg", level, __FILE__, __LINE__, m)
#define GZ_MSG1(level, m, a)\
  gz_error_print("msg", level, __FILE__, __LINE__, m, a)
#define GZ_MSG2(level, m, a, b)\
  gz_error_print("msg", level, __FILE__, __LINE__, m, a, b)
#define GZ_MSG3(level, m, a, b, c)\
  gz_error_print("msg", level, __FILE__, __LINE__, m, a, b, c)
     
#endif
