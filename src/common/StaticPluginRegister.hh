/*
 * Copyright 2011 Nate Koenig & Andrew Howard
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
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
