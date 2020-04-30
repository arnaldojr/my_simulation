

for D in creeper*; do
    if [ -d "$D" ]; then
        cd "$D"
        rm -rf materials/Creeper* 
	echo "done"
        cd ..
    fi
done



