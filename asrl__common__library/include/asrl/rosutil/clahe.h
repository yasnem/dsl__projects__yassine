#ifndef ASRL_CLAHE_HPP
#define ASRL_CLAHE_HPP

#define BYTE_IMAGE
 
#ifdef BYTE_IMAGE
typedef unsigned char kz_pixel_t;	 /* for 8 bit-per-pixel images */
#define uiNR_OF_GREY (256)
#else
//typedef unsigned short kz_pixel_t;	 /* for 12 bit-per-pixel images (default) */
//# define uiNR_OF_GREY (4096)
#endif


 
/******** Prototype of CLAHE function. Put this in a separate include file. *****/ 
int CLAHE(kz_pixel_t* pImage, unsigned int uiXRes, unsigned int uiYRes, kz_pixel_t Min, 
	  kz_pixel_t Max, unsigned int uiNrX, unsigned int uiNrY, 
	  unsigned int uiNrBins, float fCliplimit); 
 
/*********************** Local prototypes ************************/ 
 void ClipHistogram (unsigned long*, unsigned int, unsigned long); 
 void MakeHistogram (kz_pixel_t*, unsigned int, unsigned int, unsigned int, 
		unsigned long*, unsigned int, kz_pixel_t*); 
 void MapHistogram (unsigned long*, kz_pixel_t, kz_pixel_t, 
	       unsigned int, unsigned long); 
 void MakeLut (kz_pixel_t*, kz_pixel_t, kz_pixel_t, unsigned int); 
 void Interpolate (kz_pixel_t*, int, unsigned long*, unsigned long*, 
	unsigned long*, unsigned long*, unsigned int, unsigned int, kz_pixel_t*); 
 
/**************	 Start of actual code **************/ 
//#include 			 /* To get prototypes of malloc() and free() */ 
 
const unsigned int uiMAX_REG_X = 16;	  /* max. # contextual regions in x-direction */ 
const unsigned int uiMAX_REG_Y = 16;	  /* max. # contextual regions in y-direction */ 



#endif /* ASRL_CLAHE_HPP */

