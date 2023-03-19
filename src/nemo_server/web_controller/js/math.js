function quaternionToEuler(x, y, z, w){
    const ex = Math.atan2(2 * (x * y + z * w), 1 - (2 * (y * y + z * z)));
      
    const ey = Math.asin(2 * (x * z - w * y));
      
    const ez = Math.atan2(2 * (x * w + y * z), 1 - (2  * (z * z + w * w)));

    const euler = [ex, ey, ez];

    return(euler);
}