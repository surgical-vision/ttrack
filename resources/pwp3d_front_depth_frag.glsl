#version 110

varying float depth;

void main(){

	gl_FragData[0] = vec4(depth, depth, depth, depth);

}