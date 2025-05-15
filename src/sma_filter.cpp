#include "../include/sma_filter.h"

// Variables del filtro
float _filter[ORDER];
int _count;

void initFilter()
{
    for (int i = 0; i < ORDER; i++)
    {
        _filter[i] = 0;
    }
    _count = 0;
}

int getFilteredValue(int value)
{
    if (_count == 0)
    {
        // Primera iteración, asignar valor directo
        _filter[_count++] = value;
    }
    else
    {
        float alpha = ALPHA;
        for (int i = ORDER - 1; i > 0; i--)
        {
            _filter[i] = _filter[i - 1];
        }
        _filter[0] = (1 - alpha) * _filter[0] + alpha * value;
        if (_count >= ORDER)
        {
            // Mantener el filtro en el orden deseado
            for (int i = 0; i < ORDER - 1; i++)
            {
                _filter[i] = _filter[i + 1];
            }
            _count--;
        }
        else
        {
            _count++;
        }
    }
    int filteredValue = (int)_filter[ORDER - 1];
    return filteredValue;
}


