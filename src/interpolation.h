// Interpolation and filrering functions
#ifndef INTERPOLAION_H
#define INTERPOLAION_H

#include <Arduino.h>
#include "constants.h"

// Float to 0..255
inline int mapf(float in, float a, float b) __attribute__((always_inline));
inline int mapf(float in, float a, float b)
{
  if (in < a) return 0;
  if (in > b) return 255;
  return (int)(in - a) * 255 / (b - a);
}

// Linear interpolation
inline void interpolate(float *data, uint16_t *out, int matrixX, int matrixY, int imageW, int imageH, float tempRangeMin, float tempRangeMax) __attribute__((always_inline));
inline void interpolate(float *data, uint16_t *out, int matrixX, int matrixY, int imageW, int imageH, float tempRangeMin, float tempRangeMax)
{
  int scaleX = imageW / matrixX;
  int scaleY = imageH / matrixY;

  for (uint8_t h = 0; h < matrixY; h++)
  {
    for (uint8_t w = 0; w < matrixX; w++)
    {
      out[h * scaleY * imageW + w * scaleX] = mapf(data[h * matrixX + w], tempRangeMin, tempRangeMax);
    }
  }
  for (int h = 0; h < imageH; h += scaleY)
  {
    for (int w = 1; w < (imageW - scaleX); w += scaleX)
    {
      for (int i = 0; i < (scaleX - 1); i++)
      {
        out[h * imageW + w + i] = (out[h * imageW + w - 1] * ((scaleX - 1) - i) + out[h * imageW + w + (scaleX - 1)] * (i + 1)) / scaleX;
      }
    }
    for (int i = 0; i < (scaleX - 1); i++)
    {
      out[h * imageW + ((imageW - scaleX) + 1) + i] = out[h * imageW + (imageW - scaleX)];
    }
  }
  for (int w = 0; w < imageW; w++)
  {
    for (int h = 1; h < (imageH - scaleY); h += scaleY)
    {
      for (int i = 0; i < (scaleX - 1); i++)
      {
        out[(h + i) * imageW + w] = (out[(h - 1) * imageW + w] * ((scaleX - 1) - i) + out[(h + (scaleY - 1)) * imageW + w] * (i + 1)) / scaleX;
      }
    }
    for (int i = 0; i < (scaleX - 1); i++)
    {
      out[((imageH - scaleY + 1) + i) * imageW + w] = out[(imageH - scaleY) * imageW + w];
    }
  }
  for (int h = 0; h < imageH; h++)
  {
    for (int w = 0; w < imageW; w++)
    {
      out[h * imageW + w] = colorMap[out[h * imageW + w]];
    }
  }
}

// Filter (soften) temperature transition from frame to frame
inline void filter(const float *in, float *out, int matrixX, int matrixY, bool mirroring, bool filtering) __attribute__((always_inline));
inline void filter(const float *in, float *out, int matrixX, int matrixY, bool mirroring, bool filtering)
{
  if (mirroring)
  {
    for (int i = 0; i < (matrixX * matrixY); i++)
    {
      if (filtering)
        out[i] = (out[i] + in[i]) / 2;
      else
        out[i] = in[i];
    }
  }
  else
  {
    for (int i = 0; i < matrixY; i++)
      for (int j = 0; j < matrixX; j++)
      {
        if (filtering)
          out[matrixX * i + (matrixX - 1) - j] = (out[matrixX * i + (matrixX - 1) - j] + in[matrixX * i + j]) / 2;
        else
          out[matrixX * i + (matrixX - 1) - j] = in[matrixX * i + j];
      }
  }
}

#endif // INTERPOLAION_H