#ifndef BUILD_TIMESTAMP 
#warning Mercurial failed. Repository not found. Firmware revision will not be generated. 
#define HGREV N/A 
#define BUILD_TIMESTAMP "2019-05-30 03:38 UT"
#define HGREVSTR(s) stringify_(s) 
#define stringify_(s) #s 
#endif 
