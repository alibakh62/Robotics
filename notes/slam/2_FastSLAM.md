# Introduction

In SLAM, we'll map the environment given the noisy measurements and localize the robot relative to its own map giving the controls. This makes it a much more difficult problem than localization or mapping since both the map and the poses are now unknown to you. 

In real-world environments, we will primarily be faced with with SLAM problems and we will aim to estimate the map and localize the robot. 

An exmaple of a robot solving its SLAM problem is a robotic vacuum cleaner that uses the measurements provided by its laser finder sensors and data from the encoders to estimate the map and localize itself relative to it. 

## Online SLAM
