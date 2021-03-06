/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#include <gui_generated/gamescreen_screen/GameScreenViewBase.hpp>
#include <touchgfx/Color.hpp>
#include <texts/TextKeysAndLanguages.hpp>

GameScreenViewBase::GameScreenViewBase()
{

    touchgfx::CanvasWidgetRenderer::setupBuffer(canvasBuffer, CANVAS_BUFFER_SIZE);

    __background.setPosition(0, 0, 320, 240);
    __background.setColor(touchgfx::Color::getColorFrom24BitRGB(0, 0, 0));

    background.setPosition(0, 0, 320, 240);
    background.setColor(touchgfx::Color::getColorFrom24BitRGB(0, 0, 0));

    ball.setPosition(153, 113, 10, 10);
    ball.setColor(touchgfx::Color::getColorFrom24BitRGB(255, 255, 255));

    a_position.setPosition(49, 208, 92, 25);
    a_position.setVisible(false);
    a_position.setColor(touchgfx::Color::getColorFrom24BitRGB(255, 255, 255));
    a_position.setLinespacing(0);
    a_positionBuffer[0] = 0;
    a_position.setWildcard(a_positionBuffer);
    a_position.setTypedText(touchgfx::TypedText(T_SINGLEUSEID1));

    b_position.setPosition(175, 208, 92, 25);
    b_position.setVisible(false);
    b_position.setColor(touchgfx::Color::getColorFrom24BitRGB(255, 255, 255));
    b_position.setLinespacing(0);
    b_positionBuffer[0] = 0;
    b_position.setWildcard(b_positionBuffer);
    b_position.setTypedText(touchgfx::TypedText(T_SINGLEUSEID2));

    paddleA.setPosition(15, 95, 15, 50);
    paddleAPainter.setColor(touchgfx::Color::getColorFrom24BitRGB(15, 102, 240));
    paddleA.setPainter(paddleAPainter);
    paddleA.setStart(7, 5);
    paddleA.setEnd(7, 45);
    paddleA.setLineWidth(10);
    paddleA.setLineEndingStyle(touchgfx::Line::ROUND_CAP_ENDING);

    paddleB.setPosition(290, 95, 15, 50);
    paddleBPainter.setColor(touchgfx::Color::getColorFrom24BitRGB(219, 19, 19));
    paddleB.setPainter(paddleBPainter);
    paddleB.setStart(7, 5);
    paddleB.setEnd(7, 45);
    paddleB.setLineWidth(10);
    paddleB.setLineEndingStyle(touchgfx::Line::ROUND_CAP_ENDING);

    placarA.setPosition(101, 8, 40, 24);
    placarA.setColor(touchgfx::Color::getColorFrom24BitRGB(255, 255, 255));
    placarA.setLinespacing(0);
    Unicode::snprintf(placarABuffer, PLACARA_SIZE, "%s", touchgfx::TypedText(T_SINGLEUSEID8).getText());
    placarA.setWildcard(placarABuffer);
    placarA.setTypedText(touchgfx::TypedText(T_SINGLEUSEID3));

    placarB.setPosition(175, 8, 40, 24);
    placarB.setColor(touchgfx::Color::getColorFrom24BitRGB(255, 255, 255));
    placarB.setLinespacing(0);
    Unicode::snprintf(placarBBuffer, PLACARB_SIZE, "%s", touchgfx::TypedText(T_SINGLEUSEID6).getText());
    placarB.setWildcard(placarBBuffer);
    placarB.setTypedText(touchgfx::TypedText(T_SINGLEUSEID5));

    textPlacar.setPosition(141, 8, 34, 24);
    textPlacar.setColor(touchgfx::Color::getColorFrom24BitRGB(255, 255, 255));
    textPlacar.setLinespacing(0);
    textPlacar.setTypedText(touchgfx::TypedText(T_SINGLEUSEID7));

    add(__background);
    add(background);
    add(ball);
    add(a_position);
    add(b_position);
    add(paddleA);
    add(paddleB);
    add(placarA);
    add(placarB);
    add(textPlacar);
}

void GameScreenViewBase::setupScreen()
{

}
