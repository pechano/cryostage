macro "FastRAW -> EDM results v5 <---" {
dir = getDirectory("Input directory"); 
list = getFileList(dir);
setBatchMode(true);
for (i = 0; i < list.length; i++) {
	showProgress(i+1, list.length);
filename = dir + list[i];
 if (endsWith(filename, "avi")) {
run("AVI...", "select=["+dir+list[i]+"] convert"); 
  name = getTitle; 
  index = lastIndexOf(name, "."); 
  if (index!=-1) name = substring(name, 0, index); 
  name = name + ".txt"; 
run("Enhance Contrast...", "saturated=20 normalize process_all");
run("Find Edges", "stack");
setAutoThreshold("Default");
setThreshold(0, 200);
setOption("BlackBackground", false);
run("Convert to Mask", "method=Default background=Light");
run("Despeckle", "stack");
setOption("BlackBackground", false);
run("Dilate", "stack");
run("Distance Map", "stack");
run("Select All");
roiManager("Add");
roiManager("Multi Measure");
roiManager("Select", 0);
roiManager("Delete");
saveAs("Results", dir+name);
run("Close All");
		}
	}
}