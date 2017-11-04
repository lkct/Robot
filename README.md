# get the repo
`git clone git@git.posslab.space:robot-course/robot-lib.git Robot`


# dependancy
1. `sudo apt-get install libqt5serialport5-dev libopenni2-dev libudev-dev`
2. install URG
  1. `git clone git@git.posslab.space:robot-course/liburg.git`
  2. `cd liburg`
  3. `sudo make install`

# get privilege of serialport
`sudo gpasswd --add ${USER} dialout` 

# finish all
Now you can compile the codes!Lib for Robot Course.
