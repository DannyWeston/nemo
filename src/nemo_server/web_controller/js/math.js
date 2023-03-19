// Takes a quaternion and returns the 3D euler angles in Radians
function quaternionToEuler(x, y, z, w){
    const ex = Math.atan2(2 * (x * y + z * w), 1 - (2 * (y * y + z * z)));
      
    const ey = Math.asin(2 * (x * z - w * y));
      
    const ez = Math.atan2(2 * (x * w + y * z), 1 - (2  * (z * z + w * w)));

    const euler = [ex, ey, ez];

    return(euler);
}

// Takes normalised euler angles as radians for input and returns a quaternion representation
function eulerToQuaternion(x, y, z) {
    const q0 = (Math.cos(x / 2) * Math.cos(y / 2) * Math.cos(z / 2) + Math.sin(x / 2) * Math.sin(y / 2) * Math.sin(z / 2));
  
    const q1 = (Math.sin(x / 2) * Math.cos(y / 2) * Math.cos(z / 2) - Math.cos(x / 2) * Math.sin(y / 2) * Math.sin(z / 2));
  
    const q2 = (Math.cos(x / 2) * Math.sin(y / 2) * Math.cos(z / 2) + Math.sin(x / 2) * Math.cos(y/ 2) * Math.sin(z / 2));
  
    const q3 = (Math.cos(x / 2) * Math.cos(y / 2) * Math.sin(z / 2) - Math.sin(x / 2) * Math.sin(y/ 2) * Math.cos(z / 2));
  
    const quat = [q0, q1, q2, q3];
  
    return(quat);
}