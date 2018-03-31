# CanYouDoWhatICanDo

An interactive art installation game, developed during the [Rethink Hackathon](https://opencodes.io/) at the [ZKM](http://zkm.de/)'s [Open Codes exhibition](https://open-codes.zkm.de/de).

Winner of [CAS Software AG](cas.de)'s "Connecting People" Category.

[Presentation Slides](https://drive.google.com/open?id=1Tq7nP9zQ-TWXXa1uI2v78fpqDe-K0ahpxvQa5XXzbug)

Video:  
<a href="https://www.facebook.com/CASSoftware/videos/2330503683642273/
"><img height=200 src="https://i.imgur.com/Gg3GbN9.png"></a>

[DevPost](https://devpost.com/software/canyoudowhaticando)



## Possible Improvements:
- Measure size of player and adapt levels so that children can also play
- Add some sound effects
- Test on RPi (or other small plattform)


## CleanUp:
As the game was developed within a 24 hours, the code quality can be improved at some points:

- better name ('projector' is a bit generic)
- Explanation how to set up system
    - where to position camera
    - how to estimate the camera pose and set up a TF publisher to compensate for the camera pose
    - which parameters have to changed
        - cloud_gateway: resolution, m2px
- Bargraph
    - merge functions, no 10/30 magic numbers
- self.level_base_directory 
- full launch file

