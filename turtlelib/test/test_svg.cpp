#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <sstream>
#include "turtlelib/svg.hpp"

using namespace turtlelib;

TEST_CASE("SVG draw_point and to_string")
{
    Svg svg;
    svg.draw_point(1.0, 2.0, 5.0, "blue");
    std::string svg_str = svg.to_string();
    REQUIRE(svg_str.find("<circle") != std::string::npos);
    REQUIRE(svg_str.find("cx=\"504") != std::string::npos); // 408 + 1*96
    REQUIRE(svg_str.find("cy=\"336") != std::string::npos); // 528 - 2*96
    REQUIRE(svg_str.find("r=\"5") != std::string::npos);
    REQUIRE(svg_str.find("stroke=\"blue\"") != std::string::npos);
}

TEST_CASE("SVG draw_vector and to_string")
{
    Svg svg;
    svg.draw_vector(0.0, 0.0, 1.0, 1.0, "black", 2);
    std::string svg_str = svg.to_string();
    REQUIRE(svg_str.find("<line") != std::string::npos);
    REQUIRE(svg_str.find("x1=\"408") != std::string::npos); // 408 + 0*96
    REQUIRE(svg_str.find("y1=\"528") != std::string::npos); // 528 - 0*96
    REQUIRE(svg_str.find("x2=\"504") != std::string::npos); // 408 + 1*96
    REQUIRE(svg_str.find("y2=\"432") != std::string::npos); // 528 - 1*96
    REQUIRE(svg_str.find("stroke=\"black\"") != std::string::npos);
    REQUIRE(svg_str.find("stroke-width=\"2") != std::string::npos);
}

TEST_CASE("SVG draw_frame and to_string")
{
    Svg svg;
    Transform2D tf(Vector2D{1.0, 1.0}, deg2rad(0.0));
    svg.draw_frame(tf, "A", 1, 1);
    std::string svg_str = svg.to_string();
    REQUIRE(svg_str.find("<line") != std::string::npos);
    REQUIRE(svg_str.find("stroke=\"red\"") != std::string::npos); // x-axis
    REQUIRE(svg_str.find("stroke=\"green\"") != std::string::npos); // y-axis
    REQUIRE(svg_str.find("x=\"504") != std::string::npos); // Text x position
    REQUIRE(svg_str.find("y=\"432") != std::string::npos); // Text y position
    REQUIRE(svg_str.find(">A<") != std::string::npos); // Text content
}

TEST_CASE("SVG complete output matches expected") {
    Svg svg;
    svg.draw_point(1.0, 0.5);
    svg.draw_vector(0.0, 0.0, 1.0, 0.0);
    Transform2D tf;  // identity
    svg.draw_frame(tf, "{a}");
    
    std::string expected = R"SVG(<svg width="8.500000in" height="11.000000in"  viewBox="0 0 816.000000 1056.000000" xmlns="http://www.w3.org/2000/svg">
<defs>
<marker style="overflow:visible" id="Arrow1Sstart" refX="0.0" refY="0.0" orient="auto">
<path transform="scale(0.2) translate(6,0)" style="fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt" d="M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z "/>
</marker>
</defs>
<circle cx="504" cy="480" r="3" stroke="black" fill="black" stroke-width="1" />
<line x1="408" x2="504" y1="528" y2="528" stroke="black" stroke-width="5" marker-start="url(#Arrow1Sstart)" />
<g>
<line x1="408" x2="504" y1="528" y2="528" stroke="red" stroke-width="5" marker-start="url(#Arrow1Sstart)" />
<line x1="408" x2="408" y1="528" y2="432" stroke="green" stroke-width="5" marker-start="url(#Arrow1Sstart)" />
<text x="408" y="528">{a}</text>
</g>
</svg>
)SVG";
    REQUIRE(svg.to_string() == expected);
}