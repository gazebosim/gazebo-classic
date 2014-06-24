OSM Plug-in for Gazebo
	
	Author: Tashwin Khurana
	
	Version: 1.0
	
	Description: Open Street Maps plugin for Gazebo
	             This folder contains files for building osm_plugin for gazebo simulator.
	

Dependencies:

	Python 2.7

	Mapnik:

	sudo apt-get install -y python-software-properties

	sudo add-apt-repository ppa:mapnik/v2.2.0

	sudo apt-get update

	sudo apt-get install libmapnik libmapnik-dev mapnik-utils python-mapnik


Files:


osm2dict.py

	Collects data about certain types of roads based on input coordinates from osm database and converts the information received to format that can be used to build sdf files.

dict2sdf.py

	Used to build sdf file from data received about the elements in the sdf format. 
	functionality: 
		add models to world, 
		add road element, 
		set road width, 
		add points to the road element

getMapImage.py

       Gets the image of the area required to be simulated.
       
getOsmFile.py

       Downloads the osm database of the specified area.

gz_osm.py

       Command line compatible program which combine the functionality of all the above classes and functions to output the .sdf file for gazebo. 

	usage: gz_osm.py [-h] [-f OUTFILE] [-o OSMFILE] [-O INPUTOSMFILE]
	                 [-i IMAGEFILE] [-d DIRECTORY]
	                 [-B [BOUNDINGBOX [BOUNDINGBOX ...]]] [-r] [-m] [-b] [-a]
	                 [--interactive]
	
	optional arguments:
	  -h, --help            show this help message and exit
	  -f OUTFILE, --outFile OUTFILE
	                        Output file name
	  -o OSMFILE, --osmFile OSMFILE
	                        Name of the osm file generated
	  -O INPUTOSMFILE, --inputOsmFile INPUTOSMFILE
	                        Name of the Input osm file
	  -i IMAGEFILE, --imageFile IMAGEFILE
	                        Generate and name .png image of the selected areas
	  -d DIRECTORY, --directory DIRECTORY
	                        Output directory
	  -B [BOUNDINGBOX [BOUNDINGBOX ...]], --boundingbox [BOUNDINGBOX [BOUNDINGBOX ...]]
	                        Give the bounding box for the area Format: MinLon
	                        MinLat MaxLon MaxLat
	  -r, --roads           Display Roads
	  -m, --models          Display models
	  -b, --buildings       Display buildings
	  -a, --displayAll      Display roads and models
	  --interactive         Starts the interactive version of the program

Test files:

Unit testing for each of the source files is provided in the testfiles/ folder.

Usage:

	Run gz_osm.py file

		$ python gz_osm.py 

                or 

                $ ./gz_osm.py [-h] [-f OUTFILE] [-o OSMFILE] [-i IMAGEFILE] [-d DIRECTORY]
	                 [-B [BOUNDINGBOX [BOUNDINGBOX ...]]] [-r] [-m] [-b] [-a][--interactive]
	
	Output file: outFile.sdf (default)

	Check the file on gazebo
		gazebo outFile.sdf
