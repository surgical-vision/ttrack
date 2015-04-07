#version 110

uniform sampler2D tex_fd;
//uniform sampler2D tex_col;
uniform float tex_w;
uniform float tex_h;
uniform float far;
varying float depth;

void main(void)
{
	
	gl_FragData[0] = vec4(depth, depth, depth, depth);
	
	vec2 p = gl_FragCoord.xy;

	if(texture2D(tex_fd, vec2( (p.x-1.0)/tex_w, (p.y-1.0)/tex_h ) ).x == far ||
	   texture2D(tex_fd, vec2( (p.x)/tex_w,     (p.y-1.0)/tex_h ) ).x == far ||
       texture2D(tex_fd, vec2( (p.x+1.0)/tex_w, (p.y-1.0)/tex_h ) ).x == far ||
       texture2D(tex_fd, vec2( (p.x+1.0)/tex_w, (p.y)/tex_h ) ).x     == far ||
       texture2D(tex_fd, vec2( (p.x+1.0)/tex_w, (p.y+1.0)/tex_h ) ).x == far ||
       texture2D(tex_fd, vec2( (p.x)/tex_w,     (p.y+1.0)/tex_h ) ).x == far ||
       texture2D(tex_fd, vec2( (p.x-1.0)/tex_w, (p.y+1.0)/tex_h ) ).x == far ||
       texture2D(tex_fd, vec2( (p.x-1.0)/tex_w, (p.y)/tex_h ) ).x     == far)
		gl_FragData[1] = vec4(0.0,0.0,0.0,0.0);				  
	else
		gl_FragData[1] = vec4(far,far,far,far);
	
		
}