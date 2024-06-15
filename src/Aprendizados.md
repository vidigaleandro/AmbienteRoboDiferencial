#Em numpy posso utilizar um array boleano para trabalhar outro array:
---
    disc = np.array([True, True, True, False, True])
    dist = np.array([1, 2, 3, 77, 78])
    dist[~disc] = 0
    
    >>array([1, 2, 3, 0, 78])

#Calcular distancia entre pontos de dois vetores:
---
    xy = np.array([
    [x1, y1],
    [x2, y2],
    [x3, y3],
    # ... e assim por diante
    ])

    # Diferença entre pontos consecutivos
    deltas = xy[1:] - xy[:-1]

    # Norma de cada vetor de diferença
    norms = np.linalg.norm(deltas, axis=1)