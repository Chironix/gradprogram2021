filesrc = "/home/sang/gradprogram2021/src/scout_mini_ros/scout_gazebo_sim/models/smallTable1/meshes/model.dae"

meshread = open(filesrc, "r")

lines = meshread.readlines()

meshread.close()

meshwrite = open(filesrc, "w")

delete = False
end = False

for line in lines:

    if "<lines" in line: 
        delete = True
        end = False


    if "</lines>" in line: 
        end = True
        delete = False

    if delete == False and end == False:
        meshwrite.write(line)
    end = False

meshwrite.close()