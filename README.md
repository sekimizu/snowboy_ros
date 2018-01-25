# ***** NOTICE *****
This is a ROS package for snowboy running on ROS services. All coding structures are referenced from [snowboy_ros](https://github.com/tue-robotics/snowboy_ros).

------

## REQUIREMENTS
- Ubuntu 16.04
- ROS Kinetic Kame
- [PortAudio](http://www.portaudio.com/)

## DESCRIPTIONS
- Use topic "/hotword_detection" to indicate that keyword "Hi Aqua" had been detected.

> Hotword trigger notification
> > * topic: /hotword_detection
> > * type: std_msgs/String
> > * payload: {
> >  "data": "hotword_detection"
> >  }

### How to build:
- Build snowboy with Ros catkin

  ```
  $ cd ~/catkin_ws/
  $ catkin_make
  ```

### How to run:
- Export catkin_ws to bashrc:

  ```
  $ echo "source ~/catkin_ws/devel/setup.bash" > ~/.bashrc
  $ source ~/.bashrc
  ```

- Execute snowboy launch file:

  ```
  $ roscd snowboy_ros
  $ launch launch/snowboy_ros
  ```

### Test:

- Check hotword detection:

  ```
  $ rostopic echo /hotword_detection
  ```

Once if hotword had been detected, it'll show message string from topic "/hotword_detection".

### Reference:

[snowboy_ros](https://github.com/tue-robotics/snowboy_ros)

[KiTT-AI snowboy](https://github.com/Kitt-AI/snowboy)

