/*
 * dirent.h - dirent API for Microsoft Visual Studio
 *
 * Copyright (C) 2006-2012 Toni Ronkko
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * ``Software''), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED ``AS IS'', WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL TONI RONKKO BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 */
#ifndef _GAZEBO_WIN_DIRENT_H_
#define _GAZEBO_WIN_DIRENT_H_

// Define architecture flags so we don't need to include windows.h.
// Avoiding windows.h makes it simpler to use windows sockets in conjunction
// with dirent.h.
#if !defined(_68K_) && !defined(_MPPC_) && !defined(_X86_) && \
  !defined(_IA64_) && !defined(_AMD64_) && defined(_M_IX86)
#   define _X86_
#endif
#if !defined(_68K_) && !defined(_MPPC_) && !defined(_X86_) && \
  !defined(_IA64_) && !defined(_AMD64_) && defined(_M_AMD64)
#define _AMD64_
#endif

#include <cstdint>
#include <stdio.h>
#include <stdarg.h>
#include <windef.h>
#include <winbase.h>
#include <wchar.h>
#include <string.h>
#include <stdlib.h>
#include <malloc.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>

// Indicates that d_type field is available in dirent structure
#define _DIRENT_HAVE_D_TYPE

// Indicates that d_namlen field is available in dirent structure
#define _DIRENT_HAVE_D_NAMLEN

// Entries missing from MSVC 6.0
#if !defined(FILE_ATTRIBUTE_DEVICE)
#   define FILE_ATTRIBUTE_DEVICE 0x40
#endif

// File type and permission flags for stat()
#if !defined(S_IFMT)
    // File type mask
#   define S_IFMT   _S_IFMT
#endif
#if !defined(S_IFDIR)
    // Directory
#   define S_IFDIR  _S_IFDIR
#endif
#if !defined(S_IFCHR)
    // Character device
#   define S_IFCHR  _S_IFCHR
#endif
#if !defined(S_IFFIFO)
    // Pipe
#   define S_IFFIFO _S_IFFIFO
#endif
#if !defined(S_IFREG)
    // Regular file
#   define S_IFREG  _S_IFREG
#endif
#if !defined(S_IREAD)
    // Read permission
#   define S_IREAD  _S_IREAD
#endif
#if !defined(S_IWRITE)
    // Write permission
#   define S_IWRITE _S_IWRITE
#endif
#if !defined(S_IEXEC)
    // Execute permission
#   define S_IEXEC  _S_IEXEC
#endif
#if !defined(S_IFIFO)
    // Pipe
#   define S_IFIFO _S_IFIFO
#endif
#if !defined(S_IFBLK)
    // Block device
#   define S_IFBLK   0
#endif
#if !defined(S_IFLNK)
    // Link
#   define S_IFLNK   0
#endif
#if !defined(S_IFSOCK)
    // Socket
#   define S_IFSOCK  0
#endif

#if defined(_MSC_VER)
    // Read user
#   define S_IRUSR  S_IREAD
    // Write user
#   define S_IWUSR  S_IWRITE
    // Execute user
#   define S_IXUSR  0
    // Read group
#   define S_IRGRP  0
    // Write group
#   define S_IWGRP  0
    // Execute group
#   define S_IXGRP  0
    // Read others
#   define S_IROTH  0
    // Write others
#   define S_IWOTH  0
    // Execute others
#   define S_IXOTH  0
#endif

// Maximum length of file name
#if !defined(PATH_MAX)
#   define PATH_MAX MAX_PATH
#endif
#if !defined(FILENAME_MAX)
#   define FILENAME_MAX MAX_PATH
#endif
#if !defined(NAME_MAX)
#   define NAME_MAX FILENAME_MAX
#endif

// File type flags for d_type
#define DT_UNKNOWN  0
#define DT_REG      S_IFREG
#define DT_DIR      S_IFDIR
#define DT_FIFO     S_IFIFO
#define DT_SOCK     S_IFSOCK
#define DT_CHR      S_IFCHR
#define DT_BLK      S_IFBLK
#define DT_LNK      S_IFLNK

// Macros for converting between st_mode and d_type
#define IFTODT(mode) ((mode) & S_IFMT)
#define DTTOIF(type) (type)

// File type macros.  Note that block devices, sockets and links cannot be
// distinguished on Windows and the macros S_ISBLK, S_ISSOCK and S_ISLNK are
// only defined for compatibility.  These macros should always return false
// on Windows.
#define  S_ISFIFO(mode) (((mode) & S_IFMT) == S_IFIFO)
#define  S_ISDIR(mode)  (((mode) & S_IFMT) == S_IFDIR)
#define  S_ISREG(mode)  (((mode) & S_IFMT) == S_IFREG)
#define  S_ISLNK(mode)  (((mode) & S_IFMT) == S_IFLNK)
#define  S_ISSOCK(mode) (((mode) & S_IFMT) == S_IFSOCK)
#define  S_ISCHR(mode)  (((mode) & S_IFMT) == S_IFCHR)
#define  S_ISBLK(mode)  (((mode) & S_IFMT) == S_IFBLK)

// Return the exact length of d_namlen without zero terminator
#define _D_EXACT_NAMLEN(p) ((p)->d_namlen)

// Return number of bytes needed to store d_namlen
#define _D_ALLOC_NAMLEN(p) (PATH_MAX)

#ifdef __cplusplus
extern "C"
{
#endif
  // Wide-character version
  struct _wdirent
  {
    // Always zero
    int64_t d_ino;

    // Structure size
    uint16_t d_reclen;

    // Length of name without \0
    size_t d_namlen;

    // File type
    int d_type;

    // File name
    wchar_t d_name[PATH_MAX];
  };
  typedef struct _wdirent _wdirent;

  struct _WDIR
  {
    // Current directory entry
    struct _wdirent ent;

    // Private file data
    WIN32_FIND_DATAW data;

    // True if data is valid
    int cached;

    // Win32 search handle
    HANDLE handle;

    // Initial directory name
    wchar_t *patt;
  };
  typedef struct _WDIR _WDIR;

  static _WDIR *_wopendir(const wchar_t *dirname);
  static struct _wdirent *_wreaddir(_WDIR *dirp);
  static int _wclosedir(_WDIR *dirp);
  static void _wrewinddir(_WDIR* dirp);

  // For compatibility with Symbian
# define wdirent _wdirent
# define WDIR _WDIR
# define wopendir _wopendir
# define wreaddir _wreaddir
# define wclosedir _wclosedir
# define wrewinddir _wrewinddir


  // Multi-byte character versions
  struct dirent
  {
    // Always zero
    int64_t d_ino;

    // Structure size
    uint16_t d_reclen;

    // Length of name without \0
    size_t d_namlen;

    // File type
    int d_type;

    // File name
    char d_name[PATH_MAX];
  };
  typedef struct dirent dirent;

  struct DIR
  {
    struct dirent ent;
    struct _WDIR *wdirp;
  };
  typedef struct DIR DIR;

  static DIR *opendir(const char *dirname);
  static int closedir(DIR *dirp);
  static void rewinddir(DIR* dirp);


  // Internal utility functions
  static WIN32_FIND_DATAW *dirent_first(_WDIR *dirp);
  static WIN32_FIND_DATAW *dirent_next(_WDIR *dirp);

  static int dirent_mbstowcs_s(
      size_t *pReturnValue,
      wchar_t *wcstr,
      size_t sizeInWords,
      const char *mbstr,
      size_t count);

  static int dirent_wcstombs_s(
      size_t *pReturnValue,
      char *mbstr,
      size_t sizeInBytes,
      const wchar_t *wcstr,
      size_t count);

  static void dirent_set_errno(int error);

  // Open directory stream DIRNAME for read and return a pointer to the
  // internal working area that is used to retrieve individual directory
  // entries.
  static _WDIR* _wopendir(const wchar_t *dirname)
  {
    _WDIR *dirp = NULL;
    int error;

    // Must have directory name
    if (dirname == NULL  ||  dirname[0] == '\0')
    {
      dirent_set_errno(ENOENT);
      return NULL;
    }

    // Allocate new _WDIR structure
    dirp = static_cast<_WDIR*>(malloc(sizeof(struct _WDIR)));
    if (dirp != NULL)
    {
      DWORD n;

      // Reset _WDIR structure
      dirp->handle = INVALID_HANDLE_VALUE;
      dirp->patt = NULL;
      dirp->cached = 0;

      // Compute the length of full path plus zero terminator
      n = GetFullPathNameW(dirname, 0, NULL, NULL);

      // Allocate room for absolute directory name and search pattern
      dirp->patt = static_cast<wchar_t*>(malloc(sizeof(wchar_t) * n + 16));
      if (dirp->patt)
      {
        // Convert relative directory name to an absolute one.  This
        // allows rewinddir() to function correctly even when current
        // working directory is changed between opendir() and rewinddir().
        n = GetFullPathNameW(dirname, n, dirp->patt, NULL);

        if (n > 0)
        {
          wchar_t *p;

          // Append search pattern \* to the directory name
          p = dirp->patt + n;
          if (dirp->patt < p)
          {
            switch (p[-1])
            {
              case '\\':
              case '/':
              case ':':
                // Directory ends in path separator, e.g. c:\temp\
                // NOP
                break;

              default:
                // Directory name doesn't end in path separator
                *p++ = '\\';
            }
          }

          *p++ = '*';
          *p = '\0';

          // Open directory stream and retrieve the first entry
          if (dirent_first(dirp))
          {
            // Directory stream opened successfully
            error = 0;
          }
          else
          {
            // Cannot retrieve first entry
            error = 1;
            dirent_set_errno(ENOENT);
          }
        }
        else
        {
          // Cannot retrieve full path name
          dirent_set_errno(ENOENT);
          error = 1;
        }
      }
      else
      {
        // Cannot allocate memory for search pattern
        error = 1;
      }
    }
    else
    {
      // Cannot allocate _WDIR structure
      error = 1;
    }

    // Clean up in case of error
    if (error && dirp)
    {
      _wclosedir(dirp);
      dirp = NULL;
    }

    return dirp;
  }

  // Read next directory entry.  The directory entry is returned in dirent
  // structure in the d_name field.  Individual directory entries returned by
  // this function include regular files, sub-directories, pseudo-directories
  // "." and ".." as well as volume labels, hidden files and system files.
  static struct _wdirent* _wreaddir(_WDIR *dirp)
  {
    WIN32_FIND_DATAW *datap;
    struct _wdirent *entp;

    // Read next directory entry
    datap = dirent_next(dirp);
    if (datap)
    {
      size_t n;
      DWORD attr;

      // Pointer to directory entry to return
      entp = &dirp->ent;

      // Copy file name as wide-character string.  If the file name is too
      // long to fit in to the destination buffer, then truncate file name
      // to PATH_MAX characters and zero-terminate the buffer.
      n = 0;
      while (n + 1 < PATH_MAX  &&  datap->cFileName[n] != 0)
      {
        entp->d_name[n] = datap->cFileName[n];
        n++;
      }
      dirp->ent.d_name[n] = 0;

      // Length of file name excluding zero terminator
      entp->d_namlen = n;

      // File type
      attr = datap->dwFileAttributes;
      if ((attr & FILE_ATTRIBUTE_DEVICE) != 0)
      {
        entp->d_type = DT_CHR;
      }
      else if ((attr & FILE_ATTRIBUTE_DIRECTORY) != 0)
      {
        entp->d_type = DT_DIR;
      }
      else
      {
        entp->d_type = DT_REG;
      }

      // Reset dummy fields
      entp->d_ino = 0;
      entp->d_reclen = sizeof(struct _wdirent);
    }
    else
    {
      // Last directory entry read
      entp = NULL;
    }

    return entp;
  }

  // Close directory stream opened by opendir() function.  This invalidates the
  // DIR structure as well as any directory entry read previously by
  // _wreaddir().
  static int _wclosedir(_WDIR *dirp)
  {
    int ok;
    if (dirp)
    {
      // Release search handle
      if (dirp->handle != INVALID_HANDLE_VALUE)
      {
        FindClose(dirp->handle);
        dirp->handle = INVALID_HANDLE_VALUE;
      }

      // Release search pattern
      if (dirp->patt)
      {
        free(dirp->patt);
        dirp->patt = NULL;
      }

      // Release directory structure
      free(dirp);

      // success
      ok = 0;
    }
    else
    {
      // Invalid directory stream
      dirent_set_errno(EBADF);

      // failure
      ok = -1;
    }
    return ok;
  }

  // Rewind directory stream such that _wreaddir() returns the very first
  // file name again.
  static void _wrewinddir(_WDIR* dirp)
  {
    if (dirp)
    {
      // Release existing search handle
      if (dirp->handle != INVALID_HANDLE_VALUE)
      {
        FindClose(dirp->handle);
      }

      // Open new search handle
      dirent_first(dirp);
    }
  }

  // Get first directory entry (internal)
  static WIN32_FIND_DATAW* dirent_first(_WDIR *dirp)
  {
    WIN32_FIND_DATAW *datap;

    // Open directory and retrieve the first entry
    dirp->handle = FindFirstFileW(dirp->patt, &dirp->data);
    if (dirp->handle != INVALID_HANDLE_VALUE)
    {
      // a directory entry is now waiting in memory
      datap = &dirp->data;
      dirp->cached = 1;
    }
    else
    {
      // Failed to re-open directory: no directory entry in memory
      dirp->cached = 0;
      datap = NULL;
    }
    return datap;
  }

  // Get next directory entry (internal)
  static WIN32_FIND_DATAW* dirent_next(_WDIR *dirp)
  {
    WIN32_FIND_DATAW *p;

    // Get next directory entry
    if (dirp->cached != 0)
    {
      // A valid directory entry already in memory
      p = &dirp->data;
      dirp->cached = 0;
    }
    else if (dirp->handle != INVALID_HANDLE_VALUE)
    {
      // Get the next directory entry from stream
      if (FindNextFileW (dirp->handle, &dirp->data) != FALSE)
      {
        // Got a file
        p = &dirp->data;
      }
      else
      {
        // The very last entry has been processed or an error occured
        FindClose(dirp->handle);
        dirp->handle = INVALID_HANDLE_VALUE;
        p = NULL;
      }
    }
    else
    {
      // End of directory stream reached
      p = NULL;
    }
    return p;
  }

  // Open directory stream using plain old C-string.
  static DIR* opendir(const char *dirname)
  {
    struct DIR *dirp;
    int error;

    // Must have directory name
    if (dirname == NULL  ||  dirname[0] == '\0')
    {
      dirent_set_errno(ENOENT);
      return NULL;
    }

    // Allocate memory for DIR structure
    dirp = static_cast<DIR*>(malloc(sizeof(struct DIR)));
    if (dirp)
    {
      wchar_t wname[PATH_MAX];
      size_t n;

      // Convert directory name to wide-character string
      error = dirent_mbstowcs_s(&n, wname, PATH_MAX, dirname, PATH_MAX);
      if (!error)
      {
        // Open directory stream using wide-character name
        dirp->wdirp = _wopendir(wname);
        if (dirp->wdirp)
        {
          // Directory stream opened
          error = 0;
        }
        else
        {
          // Failed to open directory stream
          error = 1;
        }
      }
      else
      {
        // Cannot convert file name to wide-character string.  This
        // occurs if the string contains invalid multi-byte sequences or
        // the output buffer is too small to contain the resulting
        // string.
        error = 1;
      }
    }
    else
    {
      // Cannot allocate DIR structure
      error = 1;
    }

    // Clean up in case of error
    if (error  &&  dirp)
    {
      free(dirp);
      dirp = NULL;
    }
    return dirp;
  }

  // Close directory stream.
  static int closedir(DIR *dirp)
  {
    int ok;
    if (dirp)
    {
      // Close wide-character directory stream
      ok = _wclosedir(dirp->wdirp);
      dirp->wdirp = NULL;

      // Release multi-byte character version
      free(dirp);
    }
    else
    {
      // Invalid directory stream
      dirent_set_errno(EBADF);

      // failure
      ok = -1;
    }
    return ok;
  }

  // Rewind directory stream to beginning.
  static void rewinddir(DIR* dirp)
  {
    // Rewind wide-character string directory stream
    _wrewinddir(dirp->wdirp);
  }

  // Convert multi-byte string to wide character string
  static int dirent_mbstowcs_s(
        size_t *pReturnValue,
        wchar_t *wcstr,
        size_t sizeInWords,
        const char *mbstr,
        size_t count)
  {
    int error;

#   if defined(_MSC_VER)  &&  _MSC_VER >= 1400
    // Microsoft Visual Studio 2005 or later
    error = mbstowcs_s(pReturnValue, wcstr, sizeInWords, mbstr, count);

#   else

    // Older Visual Studio or non-Microsoft compiler
    size_t n;

    // Convert to wide-character string (or count characters)
    n = mbstowcs(wcstr, mbstr, sizeInWords);
    if (!wcstr  ||  n < count)
    {
      // Zero-terminate output buffer
      if (wcstr  &&  sizeInWords)
      {
        if (n >= sizeInWords)
        {
          n = sizeInWords - 1;
        }
        wcstr[n] = 0;
      }

      // Length of resuting multi-byte string WITH zero terminator
      if (pReturnValue)
      {
        *pReturnValue = n + 1;
      }

      // Success
      error = 0;
    }
    else
    {
      // Could not convert string
      error = 1;
    }
#endif
    return error;
  }

  // Convert wide-character string to multi-byte string
  static int dirent_wcstombs_s(
        size_t *pReturnValue,
        char *mbstr,
        // max size of mbstr
        size_t sizeInBytes,
        const wchar_t *wcstr,
        size_t count)
  {
    int error;

#   if defined(_MSC_VER)  &&  _MSC_VER >= 1400

    // Microsoft Visual Studio 2005 or later
    error = wcstombs_s(pReturnValue, mbstr, sizeInBytes, wcstr, count);

#else

    // Older Visual Studio or non-Microsoft compiler
    size_t n;

    // Convert to multi-byte string (or count the number of bytes needed)
    n = wcstombs(mbstr, wcstr, sizeInBytes);
    if (!mbstr  ||  n < count)
    {
      // Zero-terminate output buffer
      if (mbstr  &&  sizeInBytes)
      {
        if (n >= sizeInBytes)
        {
          n = sizeInBytes - 1;
        }
        mbstr[n] = '\0';
      }

      // Lenght of resulting multi-bytes string WITH zero-terminator
      if (pReturnValue)
      {
        *pReturnValue = n + 1;
      }

      // Success
      error = 0;
    }
    else
    {
      // Cannot convert string
      error = 1;
    }
#endif

    return error;
  }

  // Set errno variable
  static void dirent_set_errno(int error)
  {
#   if defined(_MSC_VER)  &&  _MSC_VER >= 1400
    // Microsoft Visual Studio 2005 and later
    _set_errno(error);
#else
    // Non-Microsoft compiler or older Microsoft compiler
    errno = error;
#endif
  }

#ifdef __cplusplus
}
#endif

// _GAZEBO_WIN_DIRENT_H_
#endif
