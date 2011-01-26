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
/* Desc: Singleton base class
 * Author: Nate Koenig
 * Date: 2 Sept 2007
 * SVN: $Id$
 */

#ifndef SINGLETONT_HH
#define SINGLETONT_HH

template < class DOOMED >
class DestroyerT;

/// \brief Singleton class
template <class T>
class SingletonT
{
  /// \brief Get an instance of the singleton
  public: static T *Instance()
          {
            if (!myself)
            {
              myself = new T();
              destroyer.SetDoomed(myself);
            }

            return myself;
          }

  /// \brief Constructor
  protected: SingletonT() {}

  /// \brief Destructor
  protected: virtual ~SingletonT() {}

  private: static DestroyerT< T > destroyer;
  private: static T *myself;
};

template <class T>
DestroyerT<T> SingletonT< T >::destroyer;

template <class T>
T *SingletonT<T>::myself = 0;



/// \brief Destroyer
template < class DOOMED >
class DestroyerT 
{

  /// \brief Constructor
  public: DestroyerT(DOOMED* = 0);

  /// \brief Destructor
  public: ~DestroyerT();

  /// \brief Sets the class to be cleaned
  public: void SetDoomed(DOOMED*);

  /// \brief Prevent users from making copies of a
  ///        Destroyer to avoid double deletion:
  private: DestroyerT(const DestroyerT<DOOMED>&);

  /// \brief Equal operator
  private: DestroyerT<DOOMED>& operator=(const DestroyerT<DOOMED>&);

  private: DOOMED* doomed;
};

template <class DOOMED>
DestroyerT<DOOMED>::DestroyerT( DOOMED* d ) 
{
  doomed = d;
}

template <class DOOMED>
DestroyerT<DOOMED>::~DestroyerT() {
  delete doomed;
}

template <class DOOMED>
void DestroyerT<DOOMED>::SetDoomed( DOOMED* d )
{
  doomed = d;
}

#endif
