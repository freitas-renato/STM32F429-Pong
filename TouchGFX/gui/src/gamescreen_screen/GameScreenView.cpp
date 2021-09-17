#include <gui/gamescreen_screen/GameScreenView.hpp>

extern "C" {
    #include "buzzer.h"
}

#define constrain(v, x, y) ((v) < (x) ? (x) : ((v) > (y) ? (y) : (v)))

GameScreenView::GameScreenView() : dx(2), dy(-1), goalA(0), goalB(0), gameStarted(false)
{

}

void GameScreenView::setupScreen()
{
    GameScreenViewBase::setupScreen();

    Unicode::snprintf(a_positionBuffer, A_POSITION_SIZE, "%d", paddleA.getY());
    Unicode::snprintf(b_positionBuffer, B_POSITION_SIZE, "%d", paddleB.getY());

}

void GameScreenView::tearDownScreen()
{
    GameScreenViewBase::tearDownScreen();
}



void GameScreenView::handleTickEvent() {
    static int tick = 0; // basic speed of the ball

    if (gameStarted) {
        if (tick++ % 2 == 0) {
            if (ball.getY() > (240 - ball_radius) || ball.getY() < (0 + ball_radius)) {
                dy *= -1;
            }

            if (ball.getX() > (320)) {  // gol na direita
                resetBall();
                dx *= -1;

                goalA += 1;
                updatePlacar();
            }

            if (ball.getX() < (0)) {  // gol na esquerda 
                resetBall();
                dx *= -1;
                dy *= -1;

                goalB += 1;
                updatePlacar();
            }

            moveBall(ball.getX() + dx, ball.getY() + dy);

            if (isTouchingB(ball.getX(), ball.getY(), paddleB.getX(), paddleB.getY()) ||
                isTouchingA(ball.getX(), ball.getY(), paddleA.getX(), paddleA.getY())) {
                dx *= -1;
                // buzzer_click();
            }
        }
    }
} 


void GameScreenView::movePaddleA(int16_t direction) {
    int16_t new_position = paddleA.getY() + (10 * direction);

    new_position = constrain(new_position, 0, 190);
    paddleA.moveTo(15, new_position);

    paddleA.invalidate();

    debugPosition();
}


void GameScreenView::movePaddleB(int16_t direction) {
    int16_t new_position = paddleB.getY() + (10 * direction);

    new_position = constrain(new_position, 0, 190);
    paddleB.moveTo(290, new_position);

    paddleB.invalidate();

    debugPosition();
}


void GameScreenView::resetPaddlePosition() {
    paddleA.moveTo(15, 95);
    paddleB.moveTo(290, 95);
    
    paddleA.invalidate();
    paddleB.invalidate();
}

void GameScreenView::moveBall(int16_t x, int16_t y) {
    ball.moveTo(x, y);
    ball.invalidate();
}

void GameScreenView::resetBall() {
    ball.moveTo(153, 113);
    ball.invalidate();
}


bool GameScreenView::isTouchingA(int16_t ball_x, int16_t ball_y, int16_t paddle_x, int16_t paddle_y) {
    return ( ((ball_x) < (paddle_x + paddle_width/2) && (ball_x) > (paddle_x)) && ((ball_y + ball_radius) > (paddle_y) && ball_y < (paddle_y + paddle_height)) );
}


bool GameScreenView::isTouchingB(int16_t ball_x, int16_t ball_y, int16_t paddle_x, int16_t paddle_y) {
    return ( ((ball_x + ball_radius) > (paddle_x - paddle_width/2) && (ball_x + ball_radius) < (paddle_x + paddle_width/2)) && ((ball_y + ball_radius) > (paddle_y) && ball_y < (paddle_y + paddle_height)) );
}

void GameScreenView::updatePlacar() {
    Unicode::snprintf(placarABuffer, PLACARA_SIZE, "%d", goalA);
    Unicode::snprintf(placarBBuffer, PLACARB_SIZE, "%d", goalB);

    placarA.invalidate();
    placarB.invalidate();

}

void GameScreenView::startGame() { 
    this->gameStarted = true;
}


void GameScreenView::debugPosition() {
    Unicode::snprintf(a_positionBuffer, A_POSITION_SIZE, "%d", paddleA.getY());
    Unicode::snprintf(b_positionBuffer, B_POSITION_SIZE, "%d", paddleB.getY());

    a_position.invalidate();
    b_position.invalidate();

}
