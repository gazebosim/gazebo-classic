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
  return 3;
}

//////////////////////////////////////////////////
void SDTS::GetData(unsigned char **_data, unsigned int &_count) const
{
  GDALRasterBand  *poBand;

  if (*_data)
    delete [] *_data;

  int nXSize = poDataset->GetRasterXSize();
  int nYSize = poDataset->GetRasterYSize();
  int nBands = poDataset->GetRasterCount();
  std::vector<float*> data_v;

  // Bands start at 1
  for (int i = 1; i <= nBands; ++i)
  {
    poBand = poDataset->GetRasterBand(i);

    // read elements
    float *buffer = new float[nXSize * nYSize];
    poBand->RasterIO(GF_Read, 0, 0, nXSize, nYSize, buffer, nXSize, nYSize, GDT_Float32, 0, 0);
    data_v.push_back(buffer);
  }

  _count = nXSize * nYSize * 3 * sizeof(unsigned char);
  *_data = new unsigned char[_count];

  unsigned char *p = *_data;

  if (nBands == 3)
  {
    for (int i = 0; i < nXSize * nYSize; ++i)
    {
      p[0] = static_cast<unsigned char>(data_v[0][i]);
      p[1] = static_cast<unsigned char>(data_v[1][i]);
      p[2] = static_cast<unsigned char>(data_v[2][i]);
      p += 3 * sizeof(unsigned char);
    }
  }
  else if (nBands == 1)
  {
    for (int i = 0; i < nXSize * nYSize; ++i)
    {
      p[0] = static_cast<unsigned char>(data_v[0][i]);
      p[1] = p[0];
      p[2] = p[0];
      p += 3 * sizeof(unsigned char);
    }
  }
  else
    std::cout << "Number of bands not supported\n";
}

//////////////////////////////////////////////////
unsigned int SDTS::GetHeight() const
{
  return this->poDataset->GetRasterYSize();
}

//////////////////////////////////////////////////
Color SDTS::GetMaxColor()
{
  Color maxClr;
  GDALRasterBand *poBand;
  double max;
  std::vector<double> max_v;

  maxClr.Set(0, 0, 0, 0);

  // read dimension sizes
  int xsize = poDataset->GetRasterXSize();
  int ysize = poDataset->GetRasterYSize();
  float *buffer = new float[xsize * ysize];
  int nBands = poDataset->GetRasterCount();

  // Bands start at 1
  for (int i = 1; i <= nBands; ++i)
  {
    // Read raster data
    poBand = poDataset->GetRasterBand(i);

    // read elements
    poBand->RasterIO(GF_Read, 0, 0, xsize, ysize, buffer, xsize, ysize, GDT_Float32, 0, 0);
    
    max = 0;
    for (int point = 0; point < xsize * ysize; ++point)
    {
      if (buffer[point] > max)
        max = buffer[point];
    }

    max_v.push_back(max);
    //std::cout << "Max: " << max << std::endl;
    //std::cout << "MAx allowed: " << poBand->GetMaximum() << std::endl;
  }

  if (max_v.size() == 1)
  {
    double v = max_v[0] / poBand->GetMaximum();
    maxClr.Set(v, v, v);
  }
  else if (max_v.size() == 3)
  {
    maxClr.Set(max_v[0] / 255.0, max_v[1] / 255.0, max_v[2] / 255.0);
  }
  else
    std::cout << "This DEM file is not supported\n";

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
