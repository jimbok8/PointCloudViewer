#include "Shader.h"

// global variables (potentially accessed by external shading functions)
float _shd_nx_c, _shd_ny_c, _shd_nz_c;
float _shd_x_c, _shd_y_c, _shd_z_c;
float _shd_vx, _shd_vy, _shd_vz;
float _shd_ndotv;
float _shd_kA, _shd_kD, _shd_kS;
unsigned char _shd_shininess;
MyDataTypes::RGBTriple _shd_specularColor;
MyDataTypes::RGBTriple _shd_Id;
MyDataTypes::RGBTriple _shd_Ir;
//void* _shd_userdata;
//int _shd_options;
//ShdLightSampleFnct _shd_lightsample;