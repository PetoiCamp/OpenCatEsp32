#ifndef _PETOIESP32SPIFFS_H
#define _PETOIESP32SPIFFS_H

#include "FS.h"
#include "SPIFFS.h"

#define FORMAT_SPIFFS_IF_FAILED true

class PetoiESP32SPIFFS{

    public:
        PetoiESP32SPIFFS();
        void FSSetup();
        void listDir(fs::FS &fs, const char * dirname, uint8_t levels);
        void readFile(fs::FS &fs, const char * path);
        void writeFile(fs::FS &fs, const char * path, const char * message);
        void appendFile(fs::FS &fs, const char * path, const char * message);
        void renameFile(fs::FS &fs, const char * path1, const char * path2);
        void deleteFile(fs::FS &fs, const char * path);

};


#endif
