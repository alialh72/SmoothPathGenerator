# SmoothPathGenerator

How to use
 - You pass the function generateSmoothPath a set of xy coordinates in the form of a Path, which contains a deque<vector<double>>
 - The function will return a Path injected with points, forming a smooth path between each coordinate
 
How it works
 - The function uses Catmull-Rom splines with an alpha of 0.75 to generate the smooth path
  
 
