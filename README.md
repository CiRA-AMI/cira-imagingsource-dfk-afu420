# cira-imagingsource-dfk-afu420

## Windows
- Install driver : https://github.com/CiRA-AMI/cira-imagingsource-dfk-afu420/raw/main/AFU420_setup_1.3.0.879.exe
  
- Install python module : 
```bash
pip install qimage2ndarray PyQt5 rospkg pythonnet numpy opencv-python
```

- Install CiRA ros-numpy : 
```bash
C:\opt\ros\melodic\x64\python.exe -m pip install https://github.com/CiRA-AMI/cira_ros_numpy/archive/refs/heads/master.zip 
```

- Run python file
```bash
python cira_imagingsource_dfk_afu420.py
```

- Use ImageCall in CiRA CORE
![image_call](https://github.com/CiRA-AMI/cira-imagingsource-dfk-afu420/blob/main/imagecall.jpg?raw=true?inline=false)
