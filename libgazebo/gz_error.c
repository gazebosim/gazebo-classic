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

/***************************************************************************
 * Desc: Error handling
 * Author: Andrew Howard
 * Date: 13 May 2002
 * CVS: $Id: gz_error.c,v 1.6.2.2 2006/12/16 22:43:22 natepak Exp $
 **************************************************************************/

#include <stdarg.h>
#include <string.h>
#include "gazebo.h"
#include "gz_error.h"

// Print to stdout?
static int error_print = 1;

// User-selected msg level: 0 for the most important messages (always
// printed); 9 for the least important.
static int error_level = 9;

// Last error
static char error_str[1024];


// Initialize error logging
void gz_error_init(int print, int level)
{
  error_print = print;
  error_level = level;
  return;
}


// Get the most recent error.
const char *gz_error_str(void)
{
  return error_str;
}


// Function for printing and logging errors.
void gz_error_print(const char *type, int level, const char *file, int line, const char *fmt, ...)
{
  va_list term_va, str_va;
  
  va_start(term_va, fmt);
  va_copy(str_va, term_va);

  if (strcmp(type, "error") == 0)
  {
    if (error_print)
    {
      fprintf(stderr, "%s:%d ", file, line);
      vfprintf(stderr, fmt, term_va);
      fprintf(stderr, "\n");
    }    
    vsnprintf(error_str, sizeof(error_str), fmt, str_va);
  }
  else
  {    
    if (level <= error_level)
    {
      if (error_print)
      {
        fprintf(stderr, "%s:%d ", file, line);
        vfprintf(stderr, fmt, term_va);
        fprintf(stderr, "\n");
      }
    }
    vsnprintf(error_str, sizeof(error_str), fmt, str_va);
  }
  
  va_end(term_va);
  va_end(str_va);
  
  return;
}
