uniform float pNear;
uniform float pFar;

varying float depth;

void main()
{
	gl_Position = ftransform();

  depth = gl_Position.z / (pFar - pNear);
}
