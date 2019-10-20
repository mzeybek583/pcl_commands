//
// PCL data import with command line for registration


int main (int argc, char** argv)
{
  if (argc < 3)
  {
    std::cerr << "No Point Cloud Data shoed. Please show source and target PCL data!" << std::endl;
    return (-1);
  }

  // Input
  if (pcl::io::loadPCDFile (argv[1], cloud_source) < 0) // Soruce data is first
  {
    std::cerr << "Failed to read source file." << std::endl;
    return (-1);
  }
  if (pcl::io::loadPCDFile (argv[2], cloud_target) < 0)
  {
    std::cerr << "Failed to read target file." << std::endl;
    return (-1);
  }
