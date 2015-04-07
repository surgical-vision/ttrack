#version 110

uniform sampler2D norm_tex;
varying float depth;
varying vec3 N;

void main(){

	gl_FragData[0] = vec4(depth, depth, depth, depth);

  gl_FragData[1] = vec4(N.x, N.y, N.z, 1);
  
}