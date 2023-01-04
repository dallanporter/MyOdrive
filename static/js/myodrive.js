(function($) {
    var myodrive = {};
    
    myodrive.name = "foobar";
    myodrive.init = () => {
        console.log("Inside myodrive.init()");
        console.log(this);
    };

    window.myodrive = myodrive;
}(jQuery));