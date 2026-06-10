document.addEventListener('DOMContentLoaded', function() {
    const circle = document.querySelector('.circle');
    let scale = 1;
    let growing = true;
    
    setInterval(() => {
        if (growing) {
            scale += 0.005;
            if (scale > 1.2) growing = false;
        } else {
            scale -= 0.005;
            if (scale < 1) growing = true;
        }
        circle.style.transform = `scale(${scale})`;
    }, 30);
});