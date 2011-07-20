

CVS_NAME=Gazebo-manual-cvs-html
REL_0_5_NAME=Gazebo-manual-0.5-html

all:
	doxygen gazebo.dox

upload_cvs: all
	./upload_www.sh $(CVS_NAME)

upload_0_5: all
	./upload_www.sh $(REL_0_5_NAME)



figures:
	-mkdir html/figures
	./makefig.py AvatarHeli 3
	./makefig.py Blimp 10
	./makefig.py CarChassis 5
	./makefig.py CanonVCC4 0.5
	./makefig.py ClodBuster 1
	./makefig.py GarminGPS 0.5
	./makefig.py MonoCam 0.5
	./makefig.py PeopleBot 2
	./makefig.py Pioneer2AT 2
	./makefig.py Pioneer2DX 2
	./makefig.py Pioneer2Gripper 0.5
	./makefig.py SegwayRMP 3
	./makefig.py Shrimp 2
	./makefig.py SickLMS200 0.5
	./makefig.py SimpleSolid 1.0
	./makefig.py SonyVID30 0.5
	./makefig.py StereoHead 0.5
	./makefig.py Bandit 1.0 
