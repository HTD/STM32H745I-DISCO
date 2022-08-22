#ifndef MODEL_HPP
#define MODEL_HPP

class ModelListener;

class Model
{
public:
    Model();

    void bind(ModelListener* listener)
    {
        modelListener = listener;
    }

    void tick();

    bool isMessagePresenterReady = false; ///< True if the current presenter can add messages to the console.
protected:
    ModelListener* modelListener;
};

#endif // MODEL_HPP
