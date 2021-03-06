#version 410

uniform bool noColor;

// Standard parameters of a VAO
in vec4 vertex;
in vec4 normal;
in vec4 color;
in vec2 texcoords;

out vec2 textCoords;
out vec4 position;

// Camera-space coordinates
out vec4 eyeVector;
out vec4 lightVector;
out vec4 lightSpace; // placeholder for shadow mapping
out vec4 vertColor;
out vec4 vertNormal;


void main( void )
{
    // You need to use color/normal/textcoords. Bad things can happen otherwise
    //if (noColor) vertColor = vec4(1., 1., 1., 1.0 );
    //else vertColor = color;
    vertColor = vec4(1., 1., 1., 1.0 );

    vertNormal = normal;
    textCoords = texcoords;

    // position
    position = vertex;
    gl_Position = vertex;
}
