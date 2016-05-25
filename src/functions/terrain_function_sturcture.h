#ifndef _TERRAIN_FUNCTION_STURCTURE_H_  
#define _TERRAIN_FUNCTION_STURCTURE_H_  

struct Feature
{
  float radius;
  float continuity_prob;
  float cross_section_prob;
  float histogram_prob;
  float normal;
  float sum;
  float slope;
};

#endif

