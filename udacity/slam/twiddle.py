from auto_go import *

def twiddle(init_params):
    n_params   = len(init_params)
    dparams    = [1.0 for row in range(n_params)]
    params     = [0.0 for row in range(n_params)]
    K = 5

    for i in range(n_params):
        params[i] = init_params[i]

    best_error = 0.0

    for k in range(K):
        ret = main(grid, init, goal, noise, params[0], params[1], params[2], params[3])
        print(params)
        print(ret[0])
        if ret[0]:
            best_error += ret[1] * 100 + ret[2]
        else:
            best_error += 99999
    best_error = float(best_error) / float(k+1)
    print(best_error)

    n = 0
    while sum(dparams) > 0.0000001:
        print('sum %d',sum(dparams))
        for i in range(len(params)):
            params[i] += dparams[i]
            err = 0
            for k in range(K):
                ret = main(grid, init, goal, noise, params[0], params[1], params[2], params[3])
                if ret[0]:
                    print(ret)
                    err += ret[1] * 100 + ret[2]
                else:
                    err += 99999
            print(float(err) / float(k+1))
            if err < best_error:
                best_error = float(err) / float(k+1)
                dparams[i] *= 1.1
            else:
                params[i] -= 2.0 * dparams[i]
                err = 0
                for k in range(K):
                    ret = main(grid, init, goal, noise, params[0], params[1], params[2], params[3])
                    if ret[0]:
                        print(ret)
                        err += ret[1] * 100 + ret[2]
                    else:
                        err += 99999
                print(float(err) / float(k+1))
                if err < best_error:
                    best_error = float(err) / float(k+1)
                    dparams[i] *= 1.1
                else:
                    params[i] += dparams[i]
                    dparams[i] *= 0.5
        n += 1
        print('Twiddle #', n, params, ' -> ', best_error)
    print(' ')
    return params

params = [weight_data, weight_smooth,p_gain, d_gain]
params = twiddle(params)
print params
 