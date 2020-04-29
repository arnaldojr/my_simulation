

for D in *_verde; do
    if [ -d "$D" ]; then
        cd "$D"
        cp -r /home/borg/catkin_ws/src/my_simulation/models/creeper_fisica/creeper_fisica/creeper11_verde/model.urdf .
	echo "done"
        cd ..
    fi
done



