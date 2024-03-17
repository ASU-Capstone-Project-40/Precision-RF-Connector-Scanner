
// Include files to use the pylon API.
#include <pylon/PylonIncludes.h>
// Extend the pylon API for using pylon data processing.
#include <pylondataprocessing/PylonDataProcessingIncludes.h>
// The sample uses the std::vector.
#include <vector>

// RecipeOutputObserver is a helper object that shows how to handle output data
// provided via the IOutputObserver::OutputDataPush interface method.
// Also, MyOutputObserver implements a thread safe queue
// for later processing while pulling the output data.
class RecipeOutputObserver : public Pylon::DataProcessing::IOutputObserver
{
public:
    RecipeOutputObserver()
            : m_waitObject(Pylon::WaitObjectEx::Create())
    {
    }

    // Implements IOutputObserver::OutputDataPush.
    // This method is called when an output of the CRecipe pushes data out.
    // The call of the method can be performed by any thread of the thread pool of the recipe.
    void OutputDataPush(
            Pylon::DataProcessing::CRecipe& recipe,
            Pylon::DataProcessing::CVariantContainer valueContainer,
            const Pylon::DataProcessing::CUpdate& update,
            intptr_t userProvidedId) override
    {
        PYLON_UNUSED(recipe);
        PYLON_UNUSED(update);
        PYLON_UNUSED(userProvidedId);

        ResultData currentResultData;
        currentResultData.fromVariantContainer(valueContainer);


        // Add data to the result queue in a thread-safe way.
        {
            Pylon::AutoLock scopedLock(m_memberLock);
            m_queue.emplace_back(currentResultData);
        }

        // Signal that data is ready.
        m_waitObject.Signal();
    }

    // Get the wait object for waiting for data.
    const Pylon::WaitObject& GetWaitObject()
    {
        return m_waitObject;
    }

    // Get one result data object from the queue.
    bool GetResultData(ResultData& resultDataOut)
    {
        Pylon::AutoLock scopedLock(m_memberLock);
        if (m_queue.empty())
        {
            return false;
        }

        resultDataOut = std::move(m_queue.front());
        m_queue.pop_front();
        if (m_queue.empty())
        {
            m_waitObject.Reset();
        }
        return true;
    }

private:
    Pylon::CLock m_memberLock;        // The member lock is required to ensure
    // thread-safe access to the member variables.
    Pylon::WaitObjectEx m_waitObject; // Signals that ResultData is available.
    // It is set if m_queue is not empty.
    std::list<ResultData> m_queue;  // The queue of ResultData
};
