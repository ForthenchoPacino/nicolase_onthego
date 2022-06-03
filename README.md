# nicolase_onthego
A shorter and hopefully slightly simplified version of the NicoLase laser array setup. The original repository can be found here: https://github.com/PRNicovich/NicoLase
.A more streamlined version of the Micro-Manager/Arduino interfacing process, including some files that were not there on the original repo.
The purpose of this re-iteration of the NicoLase project is to implement in a somehwat more simplified and easy to understand manner.
The project itself allows the user to utilize an Arduino board as a master trigger for a laser and camera array setup for fluorocense microscopy. This allows faster trigger signal transmission, as other wise, each laser would have to reace the trigger signal from the camera, thus delaying an already time-bound process.
This setup introduces the Arduino board as the master clock for the microscope apparatus, allowing signal trigger transmission with minimal overhead communication traffic and delay, allowing for more efficient result acquistion.
