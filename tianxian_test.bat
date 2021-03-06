set camera_database_path=E:\openMVG2\src\openMVG\exif\sensor_width_database\sensor_width_camera_database.txt
set excutable_dir=C:\Users\Ethan\Desktop\OPENMVG4\Windows-AMD64-Release\Release
set images_dir=C:\Users\Ethan\Desktop\jiejiaotianxian2\6m7m
set matches_dir=%images_dir%\matches
set reconstructioon_dir=%images_dir%\Global_Reconstruction


C:
cd %excutable_dir%


::openMVG_main_SfMInit_ImageListing -d %camera_database_path% -i %images_dir% -o %matches_dir%

openMVG_main_SfMInit_ImageListing -f 3800 -i %images_dir% -o %matches_dir% 


openMVG_main_ComputeFeatures -i %matches_dir%\sfm_data.json -o %matches_dir% -n 1 


openMVG_main_ComputeMatches -i %matches_dir%\sfm_data.json -o %matches_dir%  -f 1 -g e -m 1

::openMVG_main_GlobalSfM -i %matches_dir%\sfm_data.json -m %matches_dir% -o %reconstructioon_dir%

openMVG_main_tianxian -i %matches_dir%\sfm_data.json -m %matches_dir% -o %reconstructioon_dir% -a 0 -b 4 -c %images_dir%

::openMVG_main_ComputeSfM_DataColor -i %matches_dir%\sfm_data.json -o %reconstructioon_dir%\sfm_data_color.ply

::openMVG_main_ComputeStructureFromKnownPoses -i %matches_dir%\sfm_data.json -o %reconstructioon_dir%\robustFitting.json

::openMVG_main_ExportUndistortedImages -i %matches_dir%\sfm_data.json -o %reconstructioon_dir%\undistortedImages

::openMVG_main_openMVG2CMPMVS -i%reconstructioon_dir%\sfm_data.json -o %reconstructioon_dir%

::openMVG_main_openMVG2MVE -i %reconstructioon_dir%\sfm_data.json -o %reconstructioon_dir%
