
local params =
{
    batch_size = 100,
    save_dir = "picarModels",
    save_file = "model.t7",
    training_steps = 2000,
    img_height = 66,
    img_width = 200,
    img_channels = 3,
    write_summary = true,

    model_file = "Models/BinaryNet_DeepPicar_Model.lua",
    data_dir = package.searchpath("./epochs", package.path),
    out_dir = package.searchpath("./output", package.path),
    shuffle_training = true,

    train_sets = {30,32,34,36,38,40,42,44,46,48,50,52,54,56,58},
    val_sets = {31,33,35,37,39,41,43,45,47,49,51,53,55,57,59}
}

return params
