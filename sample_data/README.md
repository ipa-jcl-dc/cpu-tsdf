This data was adapted from the Augmented ICL-NUIM Dataset, as made available by Sungjoon Choi, Qian-Yi Zhou, and Vladlen Koltun at http://redwood-data.org/indoor/dataset.html, and generated as follows:

- I took a 30 frame sequence from the Living Room 1 scene; the RGB Sequence, Noisy Depth Sequence, and Ground truth
- Converted the RGB sequence from JPG to PNG via Imagemagick
- Converted the Depth + RGB PNGs to PointClouds via the pcl_png2pcd tool, e.g.
```
for f in `ls depth_pngs`; do 
  pcl_png2pcd -mode FORCE_COLOR --intensity_type FLOAT depth_pngs/$f rgb_pngs/$f output_pcds/${f%.png}.pcd; 
done
```
- Split out pose matrices from trajectory.txt into discrete files, ignoring headers. e.g.
```
i=0;
while IFS='' read -r line || [[ -n "$line" ]]; do 
  j=$((i/5));
  if [ $((i%5)) -gt 0 ]; then 
    echo "$line" >> pose_${j}.txt;
  fi;
  i=$((i+1));
done < traj.txt
```

This can be run with default parameters, but feel free to tune! Don't forget to try the --color option.

Bonus of this dataset! You can compare to ground truth PLYS, also available at the URL above.
