# plane_detection_in_PCD
Normal clustering based plane detection in a 3D point cloud
RGBD images are taken from publiclly available datasets

Title
An Efficient Clustering Algorithm to Simultaneously Detect Multiple Planes in a Point Cloud

Abstract
Abstract— Plane  detection  in  a point  cloud  is  one  of  the  primary  step  for  various applications,  such  as  computer  vision,  ground  plane  detection  for  autonomous
navigation, obstacle  detection,  indoor  scene  reconstruction,  etc.  In  this  paper,  a  new  algorithm  for simultaneous detection of multiple planes in a point cloud is 
proposed. The proposed method is  a  two-step  process.  In  the  first  step,  the  surface  normals  are  automatically  clustered  into probable plane orientations
(angular clusters) within a user specified angle threshold, without a  priori  knowledge  of  number  of  planes.  In  the  second  step,  the  angular  clusters  
are  further clustered into separate planes, within a user specified distance threshold, based on the normal distances  of  the  points  in  an  angular  cluster. 
In  contrast  to  popular  random  sampling  based methods,  the  proposed  method  uses  deterministic  approach  to  simultaneously  detect  all possible planes and has 
comparable results with the existing methods and is two times faster. The  proposed  method  is  implemented  using  Open3d  point  cloud  library  
and  evaluated  on datasets having variety of indoor scenes.  
Keywords— RGB-D, Plane Detection, Point Cloud, Open3d, Clustering, Surface Normal   

Full paper available at: 
  https://ieeexplore.ieee.org/document/9091735   

