#include "gazebo/common/SDTS.hh"

using namespace gazebo;
using namespace common;

//////////////////////////////////////////////////
SDTS::SDTS(const std::string &_filename)
{
  GDALAllRegister();

  this->poDataset = (GDALDataset *) GDALOpen(_filename.c_str(), GA_ReadOnly);

  if (this->poDataset == NULL)
    gzthrow("Unable to find SDTS file[" + _filename + "]\n");
}

//////////////////////////////////////////////////
SDTS::~SDTS()
{
}

//////////////////////////////////////////////////
unsigned int SDTS::GetBPP() const
{
  return this->poDataset->GetRasterCount();
}

//////////////////////////////////////////////////
void SDTS::GetData(unsigned char **_data, unsigned int &_count) const
{
  if (*_data)
    delete [] *_data;

  // Read raster data
  GDALRasterBand  *poBand = poDataset->GetRasterBand( 1 );  

  int nXSize = poBand->GetXSize();
  int nYSize = poBand->GetYSize();

  _count = sizeof(float) * nXSize * nYSize;

  *_data = (unsigned char *) CPLMalloc(sizeof(float) * nXSize * nYSize);

  poBand->RasterIO( GF_Read, 0, 0, nXSize, nYSize, *_data, nXSize, nYSize,
      GDT_Float32, 0, 0 );
}

//////////////////////////////////////////////////
unsigned int SDTS::GetHeight() const
{
  return this->poDataset->GetRasterYSize();
}

//////////////////////////////////////////////////
Color SDTS::GetMaxColor()
{
  unsigned int x, y;
  Color clr;
  Color maxClr;

  /*maxClr.Set(0, 0, 0, 0);

  for (y = 0; y < this->GetHeight(); y++)
  {
    for (x = 0; x < this->GetWidth(); x++)
    {
      clr = this->GetPixel(x, y);

      if (clr.r + clr.g + clr.b > maxClr.r + maxClr.g + maxClr.b)
      {
        maxClr = clr;
      }
    }
  }*/

  return maxClr;
}

//////////////////////////////////////////////////
int SDTS::GetPitch() const
{
  return this->GetWidth() * this->GetBPP();
}

//////////////////////////////////////////////////
unsigned int SDTS::GetWidth() const
{
  return this->poDataset->GetRasterXSize();
}
