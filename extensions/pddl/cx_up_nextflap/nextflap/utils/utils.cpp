#include "utils.h"
#include <ctype.h>
#include <string.h>

/********************************************************/
/* Oscar Sapena Vercher - DSIC - UPV                    */
/* April 2022                                           */
/********************************************************/
/* Constants and utilities.
 */
/********************************************************/

bool SIGNIFICATIVE_LANDMARKS = false;

#ifdef DEBUG_TO_FILE_NOT_CONSOLE
ofstream *debugFile = nullptr;
#else
ostream *debugFile = nullptr;
#endif

// Compare two strings
bool compareStr(char *s1, const char *s2) {
  unsigned int l = (unsigned int)strlen(s1);
  if (l != strlen(s2))
    return false;
  for (unsigned int i = 0; i < l; i++)
    if (tolower(s1[i]) != s2[i])
      return false;
  return true;
}

#ifdef DEBUG_TO_FILE_NOT_CONSOLE
void createDebugFile() {
  debugFile = new ofstream();
  debugFile->open("debug.txt");
}

void closeDebugFile() {
  if (debugFile != nullptr) {
    debugFile->close();
    debugFile = nullptr;
  }
}
#else
void createDebugFile() { debugFile = &cout; }

void closeDebugFile() { debugFile = nullptr; }
#endif