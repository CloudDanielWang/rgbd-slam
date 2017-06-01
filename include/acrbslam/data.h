#ifndef DATA_H
#define DATA_H

#include <acrbslam/common_include.h>

namespace acrbslam
{


class Data
{
public:


	char *red;
	char *green;
	char *blue;
	char *depth;

	//char *rotation;
	//char *translation;

	//char rotation_char[3][3];
	//char translation_char[3];

	char *rotation_char;
	char * translation_char;




};








}// namesapce acrbslam



#endif	//DATA_H