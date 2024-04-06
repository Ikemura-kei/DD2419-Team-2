import numpy as np

def global2ego(x, y, theta, x_g, y_g):
    """
    Transform a point from global to ego frame
    """
    x = np.tile(np.array([x]), (x_g.shape[0])) # (N,)
    y = np.tile(np.array([y]), (y_g.shape[0])) # (N,)
    x_b = np.cos(theta) * (x_g - x) + np.sin(theta) * (y_g - y)
    y_b = -np.sin(theta) * (x_g - x) + np.cos(theta) * (y_g - y)
    return np.stack([x_b, y_b], axis=-1) # (N, 2)

def ego2global(x, y, theta, x_b, y_b):
    """
    Transform a point from ego to global frame
    """
    x_g = np.cos(theta) * x_b - np.sin(theta) * y_b + x
    y_g = np.sin(theta) * x_b + np.cos(theta) * y_b + y
    return np.stack([x_g, y_g], axis=-1) # (N, 2

if __name__ == "__main__":
    m_g = -2
    c_g = -1

    x = 1.25
    y = -10
    theta = np.pi / 4

    # pts = global2ego(x, y, theta, np.array([0, 1]), np.array([c_g, c_g+m_g]))
    # m_b = (pts[1,1] - pts[0,1]) / (pts[1,0] - pts[0,0] + 1e-9) # (y2 - y1) / (x2 - x1)
    # c_b = pts[0,1] - m_b * pts[0,0] # y - mx
    # print(f"m_b: {m_b}, c_b: {c_b}")

    m_b = (-np.sin(theta) + np.cos(theta) * m_g) / (np.cos(theta) + np.sin(theta) * m_g)
    s = np.sin(theta)
    c = np.cos(theta)
    cg_min_y = c_g - y
    c_b = s * x + c * cg_min_y - (s * c * x - s**2 * cg_min_y - c**2 * m_g * x + s * c * m_g * cg_min_y) / (c + s * m_g)
    print(f"m_b: {m_b}, c_b: {c_b}")

    pts = ego2global(x, y, theta, np.array([0, 1]), np.array([c_b, c_b+m_b]))
    m_g_ = (pts[1,1] - pts[0,1]) / (pts[1,0] - pts[0,0] + 1e-9) # (y2 - y1) / (x2 - x1)
    c_g_ = pts[1,1] - m_g_ * pts[1,0] # y - mx
    print(f"m_g_: {m_g_}, c_g_: {c_g_}")
    print(f"m_g: {m_g}, c_g: {c_g}")




