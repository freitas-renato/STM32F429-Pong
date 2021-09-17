#ifndef GAMESCREENVIEW_HPP
#define GAMESCREENVIEW_HPP

#include <gui_generated/gamescreen_screen/GameScreenViewBase.hpp>
#include <gui/gamescreen_screen/GameScreenPresenter.hpp>

class GameScreenView : public GameScreenViewBase
{
public:
    GameScreenView();
    virtual ~GameScreenView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();

    virtual void handleTickEvent();

    void resetPaddlePosition();

    void movePaddleA(int16_t direction);
    void movePaddleB(int16_t direction);

    void moveBall(int16_t x, int16_t y);
    void resetBall();

    void startGame();

protected:

    void debugPosition();

    // Ball movement
    int16_t dx;
    int16_t dy;
    
    int goalA;
    int goalB;

    bool gameStarted;
    
    static const int16_t ball_radius = 5;
    static const int16_t paddle_width = 15;
    static const int16_t paddle_height = 50;

    bool isTouchingA(int16_t ball_x, int16_t ball_y, int16_t paddle_x, int16_t paddle_y);
    bool isTouchingB(int16_t ball_x, int16_t ball_y, int16_t paddle_x, int16_t paddle_y);

    void updatePlacar();
};

#endif // GAMESCREENVIEW_HPP
