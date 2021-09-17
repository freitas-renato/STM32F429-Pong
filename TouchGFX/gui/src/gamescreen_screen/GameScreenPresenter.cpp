#include <gui/gamescreen_screen/GameScreenView.hpp>
#include <gui/gamescreen_screen/GameScreenPresenter.hpp>

GameScreenPresenter::GameScreenPresenter(GameScreenView& v)
    : view(v)
{

}

void GameScreenPresenter::activate()
{

}

void GameScreenPresenter::deactivate()
{

}

void GameScreenPresenter::buttonEventHandler(int button_id, int button_state) {
    if (button_state == 1) {
        switch (button_id) {
            case 0:
                view.movePaddleA(-1);  // up
            break;

            case 1:
                view.movePaddleA(1);  // down
            break;

            case 2:
                view.movePaddleB(-1);  // up
            break;

            case 3:
                view.movePaddleB(1);  // down
            break;

            default:
            break;
        }
    }

    view.startGame();
}

